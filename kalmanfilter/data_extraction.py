#!/usr/bin/env python3
# ===================================================================================
# Node:         data_extraction_node
# Author:       Sophia de Souza Nobre Benevides
# Last Update:  29-10-2025
# Description:  ROS 2 node to synchronize IMU and GNSS data, transform coordinates from 
#               Geodetic (LLA) to a local Cartesian (ENU) frame, and republish the 
#               processed data for a sensor fusion algorithm.
# ===================================================================================

import rclpy
from rclpy.node import Node
import numpy as np
import message_filters
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped, Vector3Stamped, Point
from pyproj import Transformer
from cari_msgs_septentrio.msg import SeptentrioAsterx2
from icecream import ic 

# --- Coordinate Transformation Utilities ---
# LLA -> ECEF -> ENU.
# Transformer for LLA (EPSG:4326) to Earth-Centered, Earth-Fixed (EPSG:4978).
_transformer_ecef = Transformer.from_crs("EPSG:4326", "EPSG:4978", always_xy=True)

def geodetic_to_ecef(lat, lon, alt):
    """Converts geodetic coordinates (Latitude, Longitude, Altitude) to ECEF."""
    x, y, z = _transformer_ecef.transform(lon, lat, alt)
    return x, y, z

def ecef_to_enu(x, y, z, x_ref, y_ref, z_ref, lat_ref, lon_ref):
    """Converts ECEF coordinates to a local ENU (East, North, Up) frame."""
    # Translate the point to be relative to the reference origin
    dx = x - x_ref
    dy = y - y_ref
    dz = z - z_ref

    # Pre-calculate sines and cosines for the rotation matrix
    sin_lat = np.sin(np.radians(lat_ref))
    cos_lat = np.cos(np.radians(lat_ref))
    sin_lon = np.sin(np.radians(lon_ref))
    cos_lon = np.cos(np.radians(lon_ref))

    # Rotation matrix to transform from ECEF to the local ENU frame
    t = np.array([
        [-sin_lon,             cos_lon,              0],
        [-sin_lat*cos_lon, -sin_lat*sin_lon, cos_lat],
        [ cos_lat*cos_lon,  cos_lat*sin_lon, sin_lat]
    ])

    # Apply the rotation
    enu = t @ np.array([dx, dy, dz])
    return enu[0], enu[1], enu[2]

class DataFusion(Node):
    """
    Synchronizes, transforms, and republishes IMU and GNSS sensor data.
    """
    def __init__(self):
        super().__init__('data_extraction_node')

        self.ref_lat = None
        self.ref_lon = None
        self.ref_alt = None
        self.ref_ecef = None

        # --- ROS 2 Parameters for Topic Configuration ---
        self.declare_parameter('lla_topic', '/filter/positionlla')
        self.declare_parameter('imu_twist_topic', '/filter/twist')
        self.declare_parameter('gnss_pose_topic', '/poseCov')
        self.declare_parameter('septentrio_topic', '/septentrio')
        
        # --- Publishers for Processed Data ---
        self.twist_imu_pub  = self.create_publisher(TwistStamped, '/extracted_imu_twist', 10) #Imu angular and linear velocities
        self.sept_pub       = self.create_publisher(SeptentrioAsterx2, '/extracted_sept_enu', 10) #Septentrio GNSS msg
        self.enu_imu_pub    = self.create_publisher(Vector3Stamped, '/extracted_imu_enu', 10) #IMU pose data
        self.pose_gnss_pub  = self.create_publisher(PoseWithCovarianceStamped, '/extracted_gnss_pose', 10)#GNSS pose data

        # --- Message Placeholders ---
        self.twist_imu_out = TwistStamped()
        self.sept_msg_out = SeptentrioAsterx2()
        self.enu_msg_out = Vector3Stamped()
        self.pose_gnss_out = PoseWithCovarianceStamped()

        # --- Subscribers and Time Synchronizer ---
        lla_imu_sub    = message_filters.Subscriber(self, Vector3Stamped, self.get_parameter('lla_topic').value)
        twist_imu_sub  = message_filters.Subscriber(self, TwistStamped, self.get_parameter('imu_twist_topic').value)
        pose_gnss_sub  = message_filters.Subscriber(self, PoseWithCovarianceStamped, self.get_parameter('gnss_pose_topic').value)
        sept_sub       = message_filters.Subscriber(self, SeptentrioAsterx2, self.get_parameter('septentrio_topic').value)
        
        # Synchronize incoming messages from different sensors based on their timestamps.
        ts = message_filters.ApproximateTimeSynchronizer(
            [lla_imu_sub, pose_gnss_sub, twist_imu_sub, sept_sub],  
            queue_size=30,
            slop=0.5      
        )
        ts.registerCallback(self.callback)
        self.get_logger().info("Data Extraction node started and waiting for synchronized data.")
 
    def callback(self, lla_msg, pose_gnss_msg, twist_imu_msg, sept_msg):
        """Callback for synchronized messages. Transforms coordinates and republishes."""
        try:
            # Use a single timestamp for all outgoing messages to maintain sync.
            stamp = sept_msg.header.stamp

            # On the first run, set the current GNSS position as the ENU frame origin.
            if self.ref_lat is None:
                self.ref_lat = sept_msg.pose_lat
                self.ref_lon = sept_msg.pose_lon
                self.ref_alt = sept_msg.pose_alt
                self.ref_ecef = geodetic_to_ecef(self.ref_lat, self.ref_lon, self.ref_alt) 
         
            # --- Coordinate Conversion (LLA -> ECEF -> ENU) ---
            x_imu_ecef, y_imu_ecef, z_imu_ecef = geodetic_to_ecef(lla_msg.vector.x, lla_msg.vector.y, lla_msg.vector.z)
            x_imu_enu, y_imu_enu, z_imu_enu = ecef_to_enu(
                x_imu_ecef, y_imu_ecef, z_imu_ecef,
                self.ref_ecef[0], self.ref_ecef[1], self.ref_ecef[2],
                self.ref_lat, self.ref_lon
            )

            x_gnss_ecef, y_gnss_ecef, z_gnss_ecef = geodetic_to_ecef(sept_msg.pose_lat, sept_msg.pose_lon, sept_msg.pose_alt)
            x_gnss_enu, y_gnss_enu, z_gnss_enu = ecef_to_enu(
                x_gnss_ecef, y_gnss_ecef, z_gnss_ecef,
                self.ref_ecef[0], self.ref_ecef[1], self.ref_ecef[2],
                self.ref_lat, self.ref_lon
            )

            # --- Publish Processed IMU Data ---
            self.enu_msg_out.header.stamp = stamp
            self.enu_msg_out.vector.x = x_imu_enu
            self.enu_msg_out.vector.y = y_imu_enu
            self.enu_msg_out.vector.z = z_imu_enu
            self.enu_imu_pub.publish(self.enu_msg_out)

            self.twist_imu_out.header.stamp = stamp
            self.twist_imu_out.twist = twist_imu_msg.twist
            self.twist_imu_pub.publish(self.twist_imu_out)
        
            # --- Publish Processed GNSS Data ---
            #Repurposing LLA fields to carry ENU coordinates (East, North, Up).
            self.sept_msg_out.header.stamp = stamp
            self.sept_msg_out.gnss_age = sept_msg.gnss_age
            self.sept_msg_out.pose_lat = x_gnss_enu # ENU East
            self.sept_msg_out.pose_lon = y_gnss_enu # ENU North
            self.sept_msg_out.pose_alt = z_gnss_enu # ENU Up
            self.sept_pub.publish(self.sept_msg_out)

            # Publish a standard PoseWithCovarianceStamped for RViz/navigation compatibility.
            self.pose_gnss_out.header.stamp = stamp
            self.pose_gnss_out.header.frame_id = "map" # Represents the ENU frame
            self.pose_gnss_out.pose.pose.orientation = pose_gnss_msg.pose.pose.orientation
            self.pose_gnss_out.pose.pose.position = Point(x=x_gnss_enu, y=y_gnss_enu, z=z_gnss_enu)
            self.pose_gnss_pub.publish(self.pose_gnss_out)

        except Exception as e:
            self.get_logger().error(f'Error in callback: {e}', throttle_duration_sec=1)

def main(args=None):
    rclpy.init(args=args)
    node = DataFusion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Data Extraction node shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()