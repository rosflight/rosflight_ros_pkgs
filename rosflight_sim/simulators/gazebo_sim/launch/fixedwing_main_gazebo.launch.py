"""
File: fixedwing_gazebo_main.launch.py
Author: Brandon Sutherland, Jacob Moore
Created: June 22, 2023
Last Modified: July 17, 2023
Description: ROS2 launch file used to launch all parts needed for the gazebo simulation
"""

import os
import sys

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    """This is a launch file that runs the bare minimum requirements fly a fixedwing in gazebo"""

    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Whether the nodes will use sim time or not"
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    # TODO: It would be better to use the launch argument configuration instead of manually parsing
    aircraft = 'anaconda' # default aircraft
    for arg in sys.argv:
        if arg.startswith("aircraft:="):
            aircraft = arg.split(":=")[1]


    # Start simulator
    simulator_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory("rosflight_sim"),
                "launch/fixedwing_gazebo.launch.py",
            )
        ]),
        launch_arguments={
            'aircraft': aircraft,
        }.items()
    )

    # Start independent nodes
    independent_nodes_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory("rosflight_sim"),
                "launch", "independent_nodes_gazebo.launch.py",
            )
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
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

    return LaunchDescription(
        [
            use_sim_time_arg,
            simulator_launch_include,
            independent_nodes_include,
            fw_forces_moments_node,
        ]
    )
