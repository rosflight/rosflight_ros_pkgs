"""
File: fixedwing.launch.py
Author: Brandon Sutherland
Created: June 13, 2023
Last Modified: July 17, 2023
Description: ROS2 launch file used to launch Gazebo with the rosflight SIL.
"""

import os
from pathlib import Path

import xacro
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import TextSubstitution, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launches a multirotor SIL vehicle in Gazebo"""

    # Launch Arguments
    x = LaunchConfiguration('x')
    x_launch_arg = DeclareLaunchArgument(
        'x', default_value=TextSubstitution(text='0')
    )
    y = LaunchConfiguration('y')
    y_launch_arg = DeclareLaunchArgument(
        'y', default_value=TextSubstitution(text='0')
    )
    z = LaunchConfiguration('z')
    z_launch_arg = DeclareLaunchArgument(
        'z', default_value=TextSubstitution(text='0.1')
    )
    yaw = LaunchConfiguration('yaw')
    yaw_launch_arg = DeclareLaunchArgument(
        'yaw', default_value=TextSubstitution(text='0')
    )
    paused = LaunchConfiguration('paused')
    paused_launch_arg = DeclareLaunchArgument(
        'paused', default_value=TextSubstitution(text='false')
    )
    gui = LaunchConfiguration('gui')
    gui_launch_arg = DeclareLaunchArgument(
        'gui', default_value=TextSubstitution(text='true')
    )
    verbose = LaunchConfiguration('verbose')
    verbose_launch_arg = DeclareLaunchArgument(
        'verbose', default_value=TextSubstitution(text='false')
    )
    world_file = LaunchConfiguration('world_file')
    world_file_launch_arg = DeclareLaunchArgument(
        'world_file', default_value=TextSubstitution(text=os.path.join(
            get_package_share_directory('rosflight_sim'), 'resources/empty-asphalt.world'
        ))
    )
    tf_prefix = LaunchConfiguration('tf_prefix')
    tf_prefix_launch_argument = DeclareLaunchArgument(
        'tf_prefix', default_value=TextSubstitution(text="")
    )
    robot_namespace = LaunchConfiguration('robot_namespace')
    robot_namespace_launch_argument = DeclareLaunchArgument(
        'robot_namespace', default_value=TextSubstitution(text="multirotor")
    )
    gazebo_namespace = LaunchConfiguration('gazebo_namespace')
    gazebo_namespace_launch_argument = DeclareLaunchArgument(
        'gazebo_namespace', default_value=TextSubstitution(text="")
    )
    log_level = LaunchConfiguration('ros_log_level')
    log_level_launch_argument = DeclareLaunchArgument(
        'ros_log_level', default_value=TextSubstitution(text='info')
    )

    # Start simulator
    gazebo_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch/gazebo.launch.py'
            )
        ),
        launch_arguments={
            'paused': paused,
            'gui': gui,
            'verbose': verbose,
            'world': world_file,
            'params_file': os.path.join(get_package_share_directory('rosflight_sim'), 'params/multirotor_dynamics.yaml'),
        }.items()
    )

    # Render xacro file
    xacro_filepath_string = os.path.join(get_package_share_directory('rosflight_sim'), 'xacro/multirotor.urdf.xacro')
    urdf_filepath_string = os.path.join(get_package_share_directory('rosflight_sim'), 'resources/multirotor.urdf')
    robot_description = xacro.process_file(
        xacro_filepath_string, mappings={
            'mesh_file_location': os.path.join(
                get_package_share_directory('rosflight_sim'),
                'resources/multirotor.dae'
            )
        }
    ).toxml()
    Path(urdf_filepath_string).write_text(robot_description)

    # Spawn vehicle
    spawn_vehicle_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        respawn=False,
        output='screen',
        parameters=[
            {'tf_prefix': tf_prefix}
        ],
        arguments=[
            '-file', urdf_filepath_string,
            '-entity', 'robot',
            '-robot_namespace', robot_namespace,
            '-gazebo_namespace', gazebo_namespace,
            '-x', x,
            '-y', y,
            '-z', z,
            '-Y', yaw,
            '--ros-args', '--log-level', log_level
        ]
    )

    return LaunchDescription([
        x_launch_arg,
        y_launch_arg,
        z_launch_arg,
        yaw_launch_arg,
        paused_launch_arg,
        gui_launch_arg,
        verbose_launch_arg,
        world_file_launch_arg,
        tf_prefix_launch_argument,
        robot_namespace_launch_argument,
        gazebo_namespace_launch_argument,
        log_level_launch_argument,
        gazebo_launch_include,
        spawn_vehicle_node
    ])
