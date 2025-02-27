"""
File: fixedwing_standalone_io_joy.launch.py
Author: Brandon Sutherland, Jacob Moore
Created: February 3, 2025
Last Modified: February 3, 2025
Description: ROS2 launch file used to launch fixedwing SIL, rosflight_io, and rc_joy all at once.
"""

import os
import sys

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """This is a launch file that runs the bare minimum requirements fly a fixedwing in a standalone simulator"""
    aircraft = 'anaconda' # default aircraft

    # TODO: It would be better to use the launch argument configuration instead of manually parsing
    for arg in sys.argv:
        if arg.startswith("aircraft:="):
            aircraft = arg.split(":=")[1]

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
        description="Whether to use Vimfly instead of an RC node"
    )
    use_vimfly = LaunchConfiguration('use_vimfly')

    ##########
    # Launch #
    ##########

    # Start simulator
    simulator_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory("rosflight_sim"),
                "launch/fixedwing_standalone.launch.py",
            )
        ])
    )

    # Start Rosflight SIL
    rosflight_sil_node = Node(
        package="rosflight_sim",
        executable="rosflight_sil",
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

    # Start forces and moments
    fw_forces_moments_node = Node(
        package="rosflight_sim",
        executable="fixedwing_forces_and_moments",
        output="screen",
        parameters=[
            os.path.join(get_package_share_directory('rosflight_sim'),
                                 f'params/{aircraft}_dynamics.yaml'),
            {"use_sim_time": use_sim_time},
        ],
    )

    # Start dynamics node
    dynamics_node = Node(
        package="rosflight_sim",
        executable="standalone_dynamics",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}]
    )

    # Start standalone sensors
    standalone_sensor_node = Node(
        package="rosflight_sim",
        executable="standalone_sensors",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # Start rosflight_io interface node
    rosflight_io_node = Node(
        package="rosflight_io",
        executable="rosflight_io",
        output="screen",
        parameters=[{"udp": True}],
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
            simulator_launch_include,
            rosflight_sil_node,
            sil_board_node,
            fw_forces_moments_node,
            dynamics_node,
            standalone_sensor_node,
            rosflight_io_node,
            rc_joy_node,
        ]
    )
