"""
File: fixedwing_holoocean.launch.py
Author: Brandon Sutherland, Andema Mongane, Jacob Moore
Description: ROS2 launch file used to launch all the nodes to simulate a fixedwing in HoloOcean
"""

import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    dynamics_param_file_arg = DeclareLaunchArgument(
        "dynamics_param_file",
        default_value=os.path.join(get_package_share_directory('rosflight_sim'), 'params', 'anaconda_dynamics.yaml'),
        description="Parameter file that contains the dynamics of the vehicle, containing the vehicle mass parameter."
    )
    dynamics_param_file = LaunchConfiguration("dynamics_param_file")

    camera_param_file_arg = DeclareLaunchArgument(
        "camera_param_file",
        default_value=os.path.join(get_package_share_directory('rosflight_sim'),
                                   'params', 'fixedwing_down_camera.yaml'),
        description="Camera, mount, and lockstep parameter file"
    )
    env_arg = DeclareLaunchArgument(
        'env', default_value='default',
        choices=['default', 'desert', 'forest', 'island', 'mountains'],
        description='HoloOcean Land environment'
    )
    show_camera_viewer_arg = DeclareLaunchArgument(
        'show_camera_viewer', default_value='true',
        description='Open a separate viewer for the downward camera'
    )
    imu_update_frequency_arg = DeclareLaunchArgument(
        'imu_update_frequency', default_value='200.0',
        description='Simulated IMU update frequency in Hz'
    )
    flight_step_hz_arg = DeclareLaunchArgument(
        'flight_step_hz', default_value='600.0',
        description='Flight clock rate; match camera.step_hz'
    )
    use_sim_time = True
    clock_qos = {"qos_overrides./clock.subscription.reliability": "reliable"}

    ##########
    # Launch #
    ##########

    # Start simulator
    simulator_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory("rosflight_sim"),
                "launch/holoocean_sim.launch.py",
            )
        ]),
        launch_arguments={
            'agent': 'fixedwing',
            'env': LaunchConfiguration('env'),
            'camera_param_file': LaunchConfiguration('camera_param_file'),
            'imu_update_frequency': LaunchConfiguration('imu_update_frequency'),
            'lockstep': 'true',
            'use_sim_time': 'true',
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
            'use_sim_time': 'true',
            'dynamics_param_file': dynamics_param_file,
            'clock_reliability': 'reliable',
            'use_firmware_timer': 'false',
            'use_time_manager': 'false',
            'imu_frame_id': 'imu_frd',
            'imu_update_frequency': LaunchConfiguration('imu_update_frequency'),
            'clock_sync_frequency': LaunchConfiguration('flight_step_hz'),
            'rosflight_io_frame_id': 'imu_frd',
        }.items()
    )

    camera_viewer = Node(
        package='image_view',
        executable='image_view',
        name='down_camera_viewer',
        output='screen',
        remappings=[('image', '/fixedwing/camera/image_raw')],
        condition=IfCondition(LaunchConfiguration('show_camera_viewer')),
    )

    # Start forces and moments
    fw_forces_moments_node = Node(
        package="rosflight_sim",
        executable="fixedwing_forces_and_moments",
        name='fixedwing_forces_and_moments',
        output="screen",
        parameters=[
            clock_qos, {"use_sim_time": use_sim_time, "lockstep": True}, dynamics_param_file,
        ],
    )

    # Start dynamics node
    standalone_dynamics_node = Node(
        package="rosflight_sim",
        executable="standalone_dynamics",
        name='standalone_dynamics',
        output="screen",
        parameters=[clock_qos, {"use_sim_time": use_sim_time}, dynamics_param_file]
    )

    return LaunchDescription(
        [
            dynamics_param_file_arg,
            camera_param_file_arg,
            env_arg,
            show_camera_viewer_arg,
            imu_update_frequency_arg,
            flight_step_hz_arg,
            simulator_launch_include,
            common_nodes_include,
            fw_forces_moments_node,
            standalone_dynamics_node,
            camera_viewer,
        ]
    )
