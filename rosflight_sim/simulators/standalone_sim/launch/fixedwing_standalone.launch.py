"""
File: fixedwing_standalone_io_joy.launch.py
Author: Brandon Sutherland, Jacob Moore
Created: February 3, 2025
Last Modified: March 25, 2025
Description: ROS2 launch file used to launch all the nodes for a fixedwing standalone simulator
"""

import os
import sys

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition


def evaluate_namespace_arg(context, *args, **kwargs):
    # 'context' is the key here. It can evaluate substitutions.
    # 'my_arg_name' should match the name declared in DeclareLaunchArgument
    namespace = LaunchConfiguration('namespace')
    namespace_string = context.perform_substitution(namespace)

    # Now you can use 'arg_value_string' as a normal Python string
    #print(f"The evaluated argument value is: {namespace_string}")

    aircraft_body="aircraft_body"
    stl_frame="stl_frame"
    if (namespace_string != ''):
        aircraft_body=f"/{namespace_string}/aircraft_body"
        stl_frame=f"/{namespace_string}/stl_frame"
    
    # You must return a list of launch actions (or None)
    return [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            namespace=namespace,
            arguments=[
                "--x",
                '0',
                "--y",
                '0',
                "--z",
                '0',
                "--yaw",
                "-1.570796326",
                "--pitch",
                "0",
                "--roll",
                "3.1415926535",
                "--frame-id",
                aircraft_body,
                "--child-frame-id",
                stl_frame,
            ],
        )
    ]

def generate_launch_description():
    """This is a launch file that runs the bare minimum requirements fly a fixedwing in a standalone simulator"""
    dynamics_param_file_arg = DeclareLaunchArgument(
        "dynamics_param_file",
        default_value=os.path.join(get_package_share_directory('rosflight_sim'), 'params', 'anaconda_dynamics.yaml'),
        description="Parameter file that contains the dynamics of the vehicle, containing the vehicle mass parameter."
    )
    dynamics_param_file = LaunchConfiguration("dynamics_param_file")

    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Whether the nodes will use sim time or not"
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value='',
        description="Namespace for the simulation"
    )
    namespace = LaunchConfiguration('namespace')

    launch_sim_arg = DeclareLaunchArgument(
        "launch_sim",
        default_value="true",
        description="If true, launches the simulation"
    )
    launch_sim = LaunchConfiguration('launch_sim')

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
        condition=IfCondition(launch_sim),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'sim_aircraft_file': os.path.join("common_resource", "skyhunter.dae"),
            'namespace': namespace,
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
            'namespace': namespace,
        }.items()
    )

    # Start forces and moments
    fw_forces_moments_node = Node(
        package="rosflight_sim",
        executable="fixedwing_forces_and_moments",
        name='fixedwing_forces_and_moments',
        namespace=namespace,
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
        namespace=namespace,
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}, dynamics_param_file]
    )

    return LaunchDescription(
        [
            dynamics_param_file_arg,
            use_sim_time_arg,
            launch_sim_arg,
            namespace_arg,
            simulator_launch_include,
            common_nodes_include,
            fw_forces_moments_node,
            standalone_dynamics_node,
            OpaqueFunction(function=evaluate_namespace_arg),
        ]
    )
