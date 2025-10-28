"""
File: vtol_standalone.launch.py
Author: Brandon Sutherland, Jacob Moore, Mitch Messerly
Created: October 2, 2025
Last Modified: October 2, 2025
Description: ROS2 launch file used to launch all the nodes for a VTOL standalone simulator
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
    """This is a launch file that runs the bare minimum requirements fly a vtol in a standalone simulator"""
    dynamics_param_file = os.path.join(get_package_share_directory('rosflight_sim'), 'params', 'vtol_dynamics.yaml')

    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Whether the nodes will use sim time or not"
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    ##########
    # Launch #
    ##########

    # Start simulator
    simulator_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory("rosflight_sim"),
                "launch/standalone_sim.launch.py",
            )
        ]),
        launch_arguments={
            'sim_aircraft_file': os.path.join("common_resource", "skyvtol.dae") # Diff
        }.items()
    )

    # Start common nodes
    common_nodes_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("rosflight_sim"),
                "launch", "common_nodes_standalone.launch.py"
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'dynamics_param_file': dynamics_param_file,
        }.items()
    )

    # Start forces and moments
    vtol_forces_moments_node = Node(
        package="rosflight_sim",
        executable="vtol_forces_and_moments",
        name='vtol_forces_and_moments',
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time}, dynamics_param_file,
        ],
    )

    # Start dynamics node
    standalone_dynamics_node = Node(
        package="rosflight_sim",
        executable="standalone_dynamics",
        name='standalone_dynamics',
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}, dynamics_param_file]
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            simulator_launch_include,
            common_nodes_include,
            vtol_forces_moments_node,
            standalone_dynamics_node,
        ]
    )
