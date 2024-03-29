"""
File: viz_mag.launch.py
Author: Brandon Sutherland
Created: June 28, 2023
Last Modified: June 28, 2023
Description: ROS2 launch file used to launch magnetometer Rviz visualization.
"""

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import GroupAction
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    """ Launches rviz with ROSflight magnetometer visualization """

    # Get rviz config filepath
    rviz2_config_file = PathJoinSubstitution(
        [get_package_share_directory('rosflight_gcs'), 'rviz', 'viz_mag.rviz']
    )

    # Launch rviz
    rviz = GroupAction([
        PushRosNamespace('viz'),

        Node(package='rviz2',
             executable='rviz2',
             name='rviz2',
             arguments=['-d', rviz2_config_file],
             output='screen'),
    ])

    # Launch viz node
    viz_node = Node(
        package='rosflight_gcs',
        executable='viz'
    )

    return LaunchDescription([
        rviz,
        viz_node
    ])
