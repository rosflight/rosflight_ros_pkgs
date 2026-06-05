"""
File: gazebo_rf_sim.launch.py
Author: Brandon Sutherland
Created: June 15, 2023
Last Modified: July 21, 2023
Description: ROS2 launch file used to launch Gazebo with the rosflight SIL.
"""

import os
import sys
from pathlib import Path

import xacro
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import TextSubstitution, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launches a SIL vehicle in Gazebo"""

    # aircraft = LaunchConfiguration('aircraft')
    # TODO: We have to parse it in two places, the parent file and this file, since we 
    # need that info in both. LaunchConfiguration cannot be converted to a string by anything
    # other than the other launch actions... There is potentially a way to use the Command launch
    # action, which evaluates a command (i.e. evaluates the xacro command)
    # TODO: I think we could structure this so that we only need one XXXX_gazebo.launch.py....
    aircraft = 'anaconda'  # default aircraft
    aircraft_3d_file = 'skyhunter'
    for arg in sys.argv:
        if arg.startswith("aircraft:="):
            aircraft = arg.split(":=")[1]

    if aircraft == 'multirotor':
        aircraft_3d_file = 'multirotor'

    pkg_share = get_package_share_directory('rosflight_sim')

    # Launch Arguments
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')
    # gz_args = LaunchConfiguration('gz_args')
    x_launch_arg = DeclareLaunchArgument('x', default_value=TextSubstitution(text='0'))
    y_launch_arg = DeclareLaunchArgument('y', default_value=TextSubstitution(text='0'))
    z_launch_arg = DeclareLaunchArgument('z', default_value=TextSubstitution(text='0.2'))
    yaw_launch_arg = DeclareLaunchArgument('yaw', default_value=TextSubstitution(text='4.71'))
    # gz_args_launch_arg = DeclareLaunchArgument(
    #     'gz_args',
    #     default_value=TextSubstitution(text=os.path.join(pkg_share, 'gz_resource', 'runway.sdf')))

    paused = LaunchConfiguration('paused')
    # gui = LaunchConfiguration('gui')
    # verbose = LaunchConfiguration('verbose')
    # world_file = LaunchConfiguration('world_file')
    # tf_prefix = LaunchConfiguration('tf_prefix')
    paused_launch_arg = DeclareLaunchArgument('paused', default_value=TextSubstitution(text='false'))
    # gui_launch_arg = DeclareLaunchArgument('gui', default_value=TextSubstitution(text='true'))
    # verbose_launch_arg = DeclareLaunchArgument('verbose', default_value=TextSubstitution(text='false'))
    # world_file_launch_arg = DeclareLaunchArgument(
    #     'world_file', default_value=TextSubstitution(text=os.path.join(pkg_share, 'gz_resource', 'runway.sdf')))
    # tf_prefix_launch_argument = DeclareLaunchArgument('tf_prefix', default_value=TextSubstitution(text=""))
    robot_namespace = LaunchConfiguration('robot_namespace')
    robot_namespace_launch_argument = DeclareLaunchArgument(
        'robot_namespace', default_value=TextSubstitution(text='fixedwing')
    )
    # gazebo_namespace = LaunchConfiguration('gazebo_namespace')
    # gazebo_namespace_launch_argument = DeclareLaunchArgument(
    #     'gazebo_namespace', default_value=TextSubstitution(text="")
    # )
    # log_level = LaunchConfiguration('ros_log_level')
    # log_level_launch_argument = DeclareLaunchArgument(
    #     'ros_log_level', default_value=TextSubstitution(text='info')
    # )

    # resource_paths = [
    #     os.path.join(pkg_share, 'gz_resource'),
    #     os.path.join(pkg_share, 'common_resource'),
    # ]
    # resource_env = ':'.join(resource_paths)
    #
    # plugin_paths = [
    #     os.path.join(pkg_share, '..', '..', 'lib')  # install/rosflight_sim/lib
    # ]
    # plugin_env = ':'.join(plugin_paths)
    #
    # gz_sim_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
    #     ),
    #     launch_arguments={'gz_args': gz_args}.items(),
    # )

    xacro_filepath_string = os.path.join(pkg_share, f'xacro/{aircraft}.urdf.xacro')
    urdf_filepath_string = os.path.join(pkg_share, f'gz_resource/{aircraft}.urdf')
    robot_description = xacro.process_file(
        xacro_filepath_string,
        mappings={
            'mesh_file_location': os.path.join(pkg_share, f'common_resource/{aircraft_3d_file}.dae')
        }
    ).toxml()
    Path(urdf_filepath_string).write_text(robot_description)

    spawn_vehicle_node = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-world', 'default',
            '-name', 'robot',
            '-file', urdf_filepath_string,
            '-x', x,
            '-y', y,
            '-z', z,
            '-Y', yaw,
        ]
    )
    #
    # # Start simulator
    # gazebo_launch_include = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(
    #             get_package_share_directory('gazebo_ros'),
    #             'launch/gazebo.launch.py'
    #         )
    #     ),
    #     launch_arguments={
    #         'pause': paused,
    #         'gui': gui,
    #         'verbose': verbose,
    #         'world': world_file,
    #         'params_file': os.path.join(get_package_share_directory('rosflight_sim'), 'params', f'{aircraft}_dynamics.yaml')
    #     }.items()
    # )
    #
    #
    # # Spawn vehicle
    # spawn_vehicle_node = Node(
    #     package='gazebo_ros',
    #     executable='spawn_entity.py',
    #     respawn=False,
    #     output='screen',
    #     parameters=[
    #         {'tf_prefix': tf_prefix}
    #     ],
    #     arguments=[
    #         '-file', urdf_filepath_string,
    #         '-entity', 'robot',
    #         '-robot_namespace', robot_namespace,
    #         '-gazebo_namespace', gazebo_namespace,
    #         '-x', x,
    #         '-y', y,
    #         '-z', z,
    #         '-Y', yaw,
    #         '--ros-args', '--log-level', log_level
    #     ]
    # )

    return LaunchDescription([
        x_launch_arg,
        y_launch_arg,
        z_launch_arg,
        yaw_launch_arg,
        # paused_launch_arg,
        # gui_launch_arg,
        # verbose_launch_arg,
        # world_file_launch_arg,
        # tf_prefix_launch_argument,
        # gazebo_launch_include,
        # robot_namespace_launch_argument,
        # gazebo_namespace_launch_argument,
        # log_level_launch_argument,
        # gz_args_launch_arg,
        # SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_env),
        # SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', plugin_env),
        # gz_sim_launch,
        spawn_vehicle_node,
    ])
