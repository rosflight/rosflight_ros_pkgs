"""
Holocean launch file 
Authors: Braden Meyers, Andema Mongane
"""

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

    holoocean_namespace = 'holoocean'

    holoocean_main_node = Node(
        name='holoocean_node',
        package='rosflight_sim',
        executable='holoocean_node.py',
        namespace=holoocean_namespace,
        output='screen',
        emulate_tty=True,
        parameters=[
            {
                'agent': LaunchConfiguration('agent'),
                'env': LaunchConfiguration('env'),
                'show_viewport': True,
                'render_quality': -1
            }

        ],
    )

    return LaunchDescription([
        agent_arg,
        env_arg,
        holoocean_main_node,
    ])
