"""
File: swarm_example.launch.py
Author: Andrew Faucette
Created: June 24, 2026
Last Modified: June 24, 2026
Description: ROS2 launch file to serve as an example on how you can use launch arguments and namespaces
    to launch multiple UAVs
"""
import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node


def generate_launch_description():
    standalone_launch_include_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
            get_package_share_directory("rosflight_sim"),
            "launch/fixedwing_standalone.launch.py",
            )
        ]),
        # One UAV you launch must set the launch_sim argument to true (this is it's default value)
        # Each UAV must have a unique namespace declared
        # Each UAV must also have its own ports defined
        launch_arguments={
            'namespace': 'agent1',
            'launch_sim': 'true',
            'bind_port_1': '14520',
            'bind_port_2': '14521',
            }.items()
    )

    standalone_launch_include_2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
            get_package_share_directory("rosflight_sim"),
            "launch/multirotor_standalone.launch.py",
            )
        ]),
        launch_arguments={
            'namespace': 'agent2',
            'launch_sim': 'false',
            'bind_port_1': '14522',
            'bind_port_2': '14523',
            }.items()
    )

    zenoh_node = Node(
        package="rmw_zenoh_cpp",
        executable="rmw_zenohd",
        output="screen",
    )

    return LaunchDescription(
        [
            zenoh_node,
            RegisterEventHandler(
                OnProcessStart(
                    target_action=zenoh_node,
                    on_start=[
                        LogInfo(msg='Zenoh Started'),
                        standalone_launch_include_1,
                        standalone_launch_include_2,
                    ]
                )
            )
        ]
    )