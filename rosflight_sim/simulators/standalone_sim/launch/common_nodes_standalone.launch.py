"""
File: common_nodes_standalone.launch.py
Author: Jacob Moore
Created: Mar 25, 2025
Last Modified: Mar 25, 2025
Description: ROS2 launch file used to launch all nodes that are both standalone sim
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
    """This is a launch file that launches all nodes needed for a standalone simulation that do not depend on the standalone simulator"""

    rosflight_sim_dir = get_package_share_directory('rosflight_sim')
    param_file = os.path.join(rosflight_sim_dir, 'params', 'standalone_sim_params.yaml')

    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Whether the nodes will use sim time or not"
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
        output="screen",
        parameters=[{"use_sim_time": use_sim_time, "use_timer": True}],
    )

    # Start sil_board
    sil_board_node = Node(
        package="rosflight_sim",
        executable="sil_board",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # Start standalone sensors
    standalone_sensor_node = Node(
        package="rosflight_sim",
        executable="standalone_sensors",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}, param_file],
    )

    # Start dynamics node
    standalone_dynamics_node = Node(
        package="rosflight_sim",
        executable="standalone_dynamics",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}, param_file]
    )

    # Start rosflight_io interface node
    rosflight_io_node = Node(
        package="rosflight_io",
        executable="rosflight_io",
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

    # Start time manager, if applicable
    time_manager_node = Node(
        package="rosflight_sim",
        executable="standalone_time_manager",
        output="screen",
        condition=IfCondition(use_sim_time),
        parameters=[param_file]
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            use_vimfly_arg,
            rosflight_sil_node,
            sil_board_node,
            standalone_sensor_node,
            standalone_dynamics_node,
            rosflight_io_node,
            rc_joy_node,
            time_manager_node,
        ]
    )

