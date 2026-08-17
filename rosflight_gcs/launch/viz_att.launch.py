"""
File: viz_att.launch.py
Author: Brandon Sutherland
Created: June 28, 2023
Last Modified: June 28, 2023
Description: ROS2 launch file used to launch attitude Rviz visualization.
"""

from launch import LaunchDescription
from launch.actions import GroupAction
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launches rviz with ROSflight attitude visualization"""

    # Get rviz config filepath
    rviz2_config_file = PathJoinSubstitution(
        [FindPackageShare('rosflight_gcs'), 'rviz', 'viz_att.rviz']
    )

    # Launch rviz
    rviz = GroupAction(
        [
            PushRosNamespace('viz'),
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz2_config_file],
                output='screen',
            ),
        ]
    )

    # Launch viz node
    viz_node = Node(package='rosflight_gcs', executable='viz')

    return LaunchDescription([rviz, viz_node])
