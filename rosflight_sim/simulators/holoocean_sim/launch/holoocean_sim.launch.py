"""
HoloOcean launch file 
Authors: Braden Meyers, Andema Mongane
"""

import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    print('Launching HoloOcean Vehicle Simulation')

    agent_arg = DeclareLaunchArgument(
        'agent',
        default_value='fixedwing',
        description='Name of the agent to load in HoloOcean',
        choices=['multirotor', 'fixedwing']
    )

    env_arg = DeclareLaunchArgument(
        'env',
        default_value='default',
        description='Name of the environment to load in HoloOcean',
        choices=['default', 'desert', 'forest', 'island', 'mountains']
    )

    camera_param_file_arg = DeclareLaunchArgument(
        'camera_param_file',
        default_value=os.path.join(get_package_share_directory('rosflight_sim'),
                                   'params', 'fixedwing_down_camera.yaml'),
        description='Fixedwing camera and lockstep parameters'
    )
    lockstep_arg = DeclareLaunchArgument('lockstep', default_value='false')
    imu_update_frequency_arg = DeclareLaunchArgument(
        'imu_update_frequency', default_value='200.0')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')

    holoocean_namespace = 'holoocean'

    holoocean_main_node = Node(
        name='holoocean_node',
        package='rosflight_sim',
        executable='holoocean_node.py',
        namespace=holoocean_namespace,
        output='screen',
        emulate_tty=True,
        parameters=[LaunchConfiguration('camera_param_file'),
            {
                'agent': LaunchConfiguration('agent'),
                'env': LaunchConfiguration('env'),
                'show_viewport': True,
                'render_quality': -1,
                'lockstep': LaunchConfiguration('lockstep'),
                'imu_update_frequency': LaunchConfiguration('imu_update_frequency'),
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }

        ],
    )

    return LaunchDescription([
        agent_arg,
        env_arg,
        camera_param_file_arg,
        lockstep_arg,
        imu_update_frequency_arg,
        use_sim_time_arg,
        holoocean_main_node,
    ])
