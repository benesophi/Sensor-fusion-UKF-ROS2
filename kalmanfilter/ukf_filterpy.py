#!/usr/bin/env python3
# ===================================================================================
# Node:         ukf_node
# Author:       Sophia de Souza Nobre Benevides
# Last Update:  11-11-2025
# Description:  Implements an Unscented Kalman Filter (UKF) to fuse IMU (high-
#               frequency) and GNSS (low-frequency) data for vehicle localization.
# ===================================================================================

import rclpy
from rclpy.node import Node
import numpy as np
import csv
import os
from filterpy.kalman import UnscentedKalmanFilter, MerweScaledSigmaPoints
from icecream import ic
import message_filters
from geometry_msgs.msg import TwistStamped, PoseWithCovarianceStamped, Vector3Stamped
from cari_msgs_septentrio.msg import SeptentrioAsterx2
from scipy.spatial.transform import Rotation

# --- UKF State and Measurement Functions ---

def fx(chi, dt, u):
    """
    State transition function (motion model).
    Predicts the next state based on current state and control input (velocities).
    State `chi` is [x, y, z, qx, qy, qz, qw]. Control `u` is [vx, vy, vz, wx, wy, wz].
    """
    old_pos = chi[:3]
    old_quat_xyzw = chi[3:]
    v_body = u[:3]  # Linear velocities in body frame
    w_body = u[3:]  # Angular velocities in body frame

    # Update orientation using angular velocity
    rot_vec = w_body * dt
    delta_rotation = Rotation.from_rotvec(rot_vec)
    new_rotation = Rotation.from_quat(old_quat_xyzw) * delta_rotation
    new_quat_xyzw = new_rotation.as_quat()

    # Transform linear velocity from body to world frame using the old orientation
    world_velocity = Rotation.from_quat(old_quat_xyzw).apply(v_body)
    new_pos = old_pos + world_velocity * dt
    
    return np.concatenate([new_pos, new_quat_xyzw])
    
def hx(chi): 
    """Measurement function. Assumes a direct observation of the state."""
    return chi

def nearest_spd(A):
    """Ensures a matrix is Symmetric Positive Definite (SPD) for numerical stability."""
    A = 0.5 * (A + A.T)
    U, s, Vt = np.linalg.svd(A)
    s = np.clip(s, 0, None)
    A_spd = (U * s) @ Vt
    return 0.5 * (A_spd + A_spd.T)

def update_R_matrix(gnss_age, R_base):
    """
    Dynamically adjusts measurement noise `R` based on GNSS signal age.
    Older signals result in higher uncertainty (less trust in the measurement).
    """
    limit= 5.0
    max_limit = 120.0

    if gnss_age <= limit:
        factor = 1.0
    elif limit < gnss_age <= max_limit:
        progress = (gnss_age - limit) / (max_limit - limit)
        factor = 1.0 + (progress * 999.0) 
    else: # gnss_age > max_limit
        factor = 1e4 # High penalty for very old data
    
    return R_base * max(1.0, factor)

def Q_matrix():
    """Defines the process noise covariance Q."""
    Q = np.diag([
        (1)**2, (1)**2, (1)**2,          
        (np.radians(0.12))**2, (np.radians(0.12))**2, 
        (np.radians(0.12))**2, (np.radians(0.12))**2 
    ])
    return nearest_spd(Q)

def R_matrix(): 
    """Defines the base measurement noise covariance R."""
    R = np.diag([
        (0.03)**2, (0.03)**2, (0.03)**2, 
        (np.radians(0.8))**2, (np.radians(0.8))**2, 
        (np.radians(0.8))**2, (np.radians(0.8))**2 
    ])
    return nearest_spd(R)

def P_matrix(): 
    """Defines the initial state estimate covariance P."""
    P = np.diag([
        (0.1)**2, (0.1)**2, (0.1)**2, 
        (np.radians(5.0))**2, (np.radians(5.0))**2, 
        (np.radians(5.0))**2, (np.radians(5.0))**2  
    ])
    return nearest_spd(P)

class UKFNode(Node):
    def __init__(self):
        node_name = 'ukf_node_' + str(os.getpid())
        super().__init__(node_name)

        # --- ROS Parameters ---
        self.declare_parameter('twist_input_topic', '/extracted_imu_twist')
        self.declare_parameter('septentrio_topic', '/extracted_sept_enu')
        self.declare_parameter('gnss_pose_topic', '/extracted_gnss_pose')
        self.declare_parameter('imu_pose_topic', '/extracted_imu_enu')

        twist_topic = self.get_parameter('twist_input_topic').value
        septentrio_topic = self.get_parameter('septentrio_topic').value
        gnss_pose_topic = self.get_parameter('gnss_pose_topic').value
        imu_pose_topic = self.get_parameter('imu_pose_topic').value

        # --- UKF Filter Setup ---
        points = MerweScaledSigmaPoints(n=7, alpha=0.1, beta=2.0, kappa=0)
        self.ukf = UnscentedKalmanFilter(dim_x=7, dim_z=7, fx=fx, hx=hx, dt=0.01, points=points)
        self.ukf.x = np.array([0., 0., 0., 0., 0., 0., 1.]) # Initial state [x,y,z, qx,qy,qz,qw]
        self.ukf.P = P_matrix()
        self.ukf.Q = Q_matrix()
        self.Rbase = R_matrix() # Base measurement noise
        self.ukf.R = self.Rbase
        
        self.last_predict_timestamp = None
        self.filter_initialized = False # Flag to wait for the first GNSS measurement

        # --- DUAL SUBSCRIBER ARCHITECTURE ---
        #High-frequency subscriber for PREDICT step (IMU Twist)
        self.twist_subscriber = self.create_subscription(
            TwistStamped, twist_topic, self.predict_callback,
            rclpy.qos.qos_profile_sensor_data
        )

        # Low-frequency time synchronizer for UPDATE step (GNSS data)
        gnss_pose_sub = message_filters.Subscriber(self, PoseWithCovarianceStamped, gnss_pose_topic)
        septentrio_sub = message_filters.Subscriber(self, SeptentrioAsterx2, septentrio_topic)
        imu_pose_sub = message_filters.Subscriber(self, Vector3Stamped, imu_pose_topic)

        self.gnss_synchronizer = message_filters.ApproximateTimeSynchronizer(
            [gnss_pose_sub, septentrio_sub, imu_pose_sub],
            queue_size=10, slop=0.5
        )
        self.gnss_synchronizer.registerCallback(self.update_callback)

        self.get_logger().info("UKF Node ready with separate PREDICT and UPDATE callbacks.")

        # --- Log File Setup ---
        self.csv_file  = os.path.expanduser('~/ros2_ic/src/ic/kalmanfilter/results/files/kf_data.csv')
        self.gnss_tum  = os.path.expanduser('~/ros2_ic/src/ic/kalmanfilter/results/files/gnss_trajectory.tum')
        self.ukf_tum   = os.path.expanduser('~/ros2_ic/src/ic/kalmanfilter/results/files/ukf_trajectory.tum')
        self.imu_tum   = os.path.expanduser('~/ros2_ic/src/ic/kalmanfilter/results/files/imu_trajectory.tum') 

        for fname in (self.csv_file, self.gnss_tum, self.ukf_tum, self.imu_tum):
            try: os.remove(fname)
            except OSError: pass
        # CSV header
        with open(self.csv_file, 'w', newline='') as f:
            csv.writer(f).writerow(['timestamp', 'est_x','est_y','est_z','est_qx','est_qy','est_qz','est_qw',
                                     'meas_x','meas_y','meas_z','meas_qx','meas_qy','meas_qz','meas_qw',
                                     'imu_x', 'imu_y', 'imu_z', 'gnss_age'])

    def predict_callback(self, twist_msg):
        """High-frequency callback for the UKF predict step, driven by IMU data."""
        if not self.filter_initialized:
            return # Don't predict until the filter is initialized by the first update
        
        current_time = rclpy.time.Time.from_msg(twist_msg.header.stamp)
        if self.last_predict_timestamp is None:
            self.last_predict_timestamp = current_time
            return
            
        dt = (current_time - self.last_predict_timestamp).nanoseconds * 1e-9

        # Create control vector u from IMU twist message
        v = twist_msg.twist.linear
        w = twist_msg.twist.angular
        u = np.array([v.x, v.y, v.z, w.x, w.y, w.z])
        
        # Run UKF prediction
        self.ukf.predict(dt=dt, u=u)
        self.ukf.x[3:] /= np.linalg.norm(self.ukf.x[3:]) # Normalize quaternion
        
        self.last_predict_timestamp = current_time

    def update_callback(self, pose_gnss_msg, sept_msg, imu_pose_msg):
        """Low-frequency callback for the UKF update step, driven by synchronized GNSS data."""
        # The first GNSS measurement initializes the filter's state
        if not self.filter_initialized:
            pos_initial = np.array([sept_msg.pose_lat, sept_msg.pose_lon, sept_msg.pose_alt])
            orient = pose_gnss_msg.pose.pose.orientation
            quat_initial = np.array([orient.x, orient.y, orient.z, orient.w])
            self.ukf.x = np.concatenate([pos_initial, quat_initial])
            self.filter_initialized = True
            self.get_logger().info(f"Filter initialized with first GNSS measurement.")
        
        # Assemble the measurement vector 'z' from GNSS data
        pos_meas_enu = [sept_msg.pose_lat, sept_msg.pose_lon, sept_msg.pose_alt]
        q_msg = pose_gnss_msg.pose.pose.orientation
        quat_xyzw = np.array([q_msg.x, q_msg.y, q_msg.z, q_msg.w])
        z = np.concatenate([pos_meas_enu, quat_xyzw])

        # Ensure quaternion continuity (prevent 180-degree flips)
        if np.dot(self.ukf.x[3:], z[3:]) < 0:
            z[3:] *= -1
            
        gnss_age = sept_msg.gnss_age
        self.ukf.R = update_R_matrix(gnss_age, self.Rbase)
        self.ukf.update(z)
        self.ukf.x[3:] /= np.linalg.norm(self.ukf.x[3:]) # Re-normalize quaternion

        # --- Data Logging ---
        ts = rclpy.time.Time.from_msg(sept_msg.header.stamp).nanoseconds * 1e-9
        imu_p = imu_pose_msg.vector
        x_est = self.ukf.x
        with open(self.imu_tum, 'a') as f: f.write(f"{ts:.6f} {imu_p.x:.6f} {imu_p.y:.6f} {imu_p.z:.6f} 0 0 0 1\n")
        with open(self.gnss_tum, 'a') as f: f.write(f"{ts:.6f} {z[0]:.6f} {z[1]:.6f} {z[2]:.6f} {z[3]:.6f} {z[4]:.6f} {z[5]:.6f} {z[6]:.6f}\n")
        with open(self.ukf_tum, 'a') as f: f.write(f"{ts:.6f} {x_est[0]:.6f} {x_est[1]:.6f} {x_est[2]:.6f} {x_est[3]:.6f} {x_est[4]:.6f} {x_est[5]:.6f} {x_est[6]:.6f}\n")
        
        data_row = [ts, *x_est, *z, imu_p.x, imu_p.y, imu_p.z, gnss_age]
        with open(self.csv_file, 'a', newline='') as f:
            csv.writer(f).writerow(data_row)
            
        # Sync predict timestamp to avoid large dt jump after an update
        self.last_predict_timestamp = rclpy.time.Time.from_msg(sept_msg.header.stamp)

def main(args=None):
    """Initializes and runs the ROS 2 node."""
    rclpy.init(args=args)
    ukf_node = UKFNode()
    try:
        rclpy.spin(ukf_node)
    except KeyboardInterrupt:
        ukf_node.get_logger().info("UKF node shutting down.")
    finally:
        ukf_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()