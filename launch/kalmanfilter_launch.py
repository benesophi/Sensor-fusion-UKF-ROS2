# ===================================================================================
# Author:       Sophia de Souza Nobre Benevides
# Last Update:  4-12-2025
# Description:  Launches the UKF pipeline nodes and a rosbag player.
# ===================================================================================

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    """Defines and launches all processes for the UKF test."""

    path_bag = '/home/lasi/ros2_ic/src/ic/kalmanfilter/bag_ford/rosbag2_2025_06_20-15_16_31_0.db3' 

    # Node for data preprocessing and coordinate conversion.
    data_extraction_node = Node(
      package='kalmanfilter',
      executable='data_extraction_node',
      name='data_extraction_node',
      output='screen',
      parameters=[{'use_sim_time': True}] 
    )

    # Node that runs the Unscented Kalman Filter.
    ukf_node = Node(
    package='kalmanfilter',
    executable='ukf_node',
    name='ukf_node_' + str(os.getpid()),
    output='screen',
    parameters=[{'use_sim_time': True}] 
    )

    # Process to play the rosbag file at 2x speed.
    play_bag_process = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', path_bag, '--rate', '2.0'],
        output='screen'
    )

    # Return the launch description with all actions.
    return LaunchDescription([
        data_extraction_node,
        ukf_node,
        play_bag_process
    ])