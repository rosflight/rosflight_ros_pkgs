"""
File: multirotor_gazebo.launch.py
Author: Brandon Sutherland
Created: June 22, 2023
Last Modified: July 17, 2023
Description: ROS2 launch file used to launch multirotor gazebo simulator
"""

import os
import sys

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """This is a launch file that runs the bare minimum requirements fly a multirotor in gazebo"""

    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Whether the nodes will use sim time or not"
    )
    use_sim_time = LaunchConfiguration('use_sim_time')


    # Start simulator
    simulator_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rosflight_sim'),
                'launch', 'gazebo_rf_sim.launch.py'
            )
        ),
        launch_arguments={
            'robot_namespace': 'multirotor',
        }.items()
    )

    # Start independent nodes
    independent_nodes_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("rosflight_sim"),
                "launch", "common_nodes_gazebo.launch.py",
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )

    # Start forces and moments
    mr_forces_moments_node = Node(
        package="rosflight_sim",
        executable="multirotor_forces_and_moments",
        name="multirotor_forces_and_moments",
        output="screen",
        parameters=[
            os.path.join(get_package_share_directory('rosflight_sim'),
                                 f'params/multirotor_dynamics.yaml'),
            {"use_sim_time": use_sim_time},
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        simulator_launch_include,
        independent_nodes_include,
        mr_forces_moments_node
    ])
