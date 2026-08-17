"""
File: fixedwing_standalone_io_joy.launch.py
Author: Brandon Sutherland, Jacob Moore
Created: February 3, 2025
Last Modified: March 25, 2025
Description: ROS2 launch file used to launch all the nodes for a fixedwing standalone simulator
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    """This is a launch file that runs the bare minimum requirements fly a fixedwing in a standalone simulator"""

    rosflight_sim_share = FindPackageShare('rosflight_sim')

    dynamics_param_file_arg = DeclareLaunchArgument(
        "dynamics_param_file",
        default_value=PathJoinSubstitution([rosflight_sim_share, 'params', 'anaconda_dynamics.yaml']),
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

    use_vimfly_arg = DeclareLaunchArgument(
        "use_vimfly",
        default_value="false",
        description="Whether the rc node will use vimfly or not"
    )
    use_vimfly = LaunchConfiguration('use_vimfly')

    ##########
    # Launch #
    ##########

    # Start simulator
    simulator_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([rosflight_sim_share, 'launch', 'standalone_sim.launch.py'])
        ),
        launch_arguments={'sim_aircraft_file': "common_resource/skyhunter.dae"}.items()
    )

    # Start common nodes
    common_nodes_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [rosflight_sim_share, 'launch', 'common_nodes_standalone.launch.py']
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_vimfly': use_vimfly,
            'dynamics_param_file': dynamics_param_file,
        }.items()
    )

    # Start forces and moments
    fw_forces_moments_node = Node(
        package="rosflight_sim",
        executable="fixedwing_forces_and_moments",
        name='fixedwing_forces_and_moments',
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
            dynamics_param_file_arg,
            use_sim_time_arg,
            use_vimfly_arg,
            simulator_launch_include,
            common_nodes_include,
            fw_forces_moments_node,
            standalone_dynamics_node,
        ]
    )
