"""
File: independent_nodes_gazebo.launch.py
Author: Jacob Moore
Created: Mar 20, 2025
Last Modified: Mar 20, 2025
Description: ROS2 launch file used to launch all nodes that are both gazebo
    and frame-type independent.
"""

import os
import sys

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    """This is a launch file that launches all nodes needed for a gazebo simulation that do not depend on gazebo"""

    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",  # Gazebo uses sim time, but publishes at 10 Hz
        description="Whether the nodes will use sim time or not. Defaults to false for Gazebo"
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    use_vimfly_arg = DeclareLaunchArgument(
        "use_vimfly",
        default_value="false",
        description="Whether the rc node will use vimfly or not"
    )
    use_vimfly = LaunchConfiguration('use_vimfly')


    # Start Rosflight SIL
    rosflight_sil_node = Node(
        package="rosflight_sim",
        executable="rosflight_sil_manager",
        name="rosflight_sil_manager",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time, "use_timer": True}],
    )

    # Start sil_board
    sil_board_node = Node(
        package="rosflight_sim",
        executable="sil_board",
        name="sil_board",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # Start standalone sensors
    standalone_sensor_node = Node(
        package="rosflight_sim",
        executable="standalone_sensors",
        name="standalone_sensors",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # Start rosflight_io interface node
    rosflight_io_node = Node(
        package="rosflight_io",
        executable="rosflight_io",
        name="rosflight_io",
        output="screen",
        parameters=[{"udp": True,
                     "use_sim_time": use_sim_time}],
    )

    # Start rc_joy node for RC input
    rc_joy_node = Node(
        package="rosflight_sim",
        executable="rc.py",
        parameters=[{"use_vimfly": use_vimfly, "use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            use_vimfly_arg,
            rosflight_sil_node,
            sil_board_node,
            standalone_sensor_node,
            rosflight_io_node,
            rc_joy_node,
        ]
    )
