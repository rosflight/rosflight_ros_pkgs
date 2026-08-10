"""
File: common_nodes_standalone.launch.py
Author: Jacob Moore
Created: Mar 25, 2025
Last Modified: Mar 25, 2025
Description: ROS2 launch file used to launch all nodes that are both standalone sim
    and frame-type independent.
"""

import os
import sys

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import TextSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    """This is a launch file that launches all nodes needed for a standalone simulation that do not depend on the standalone simulator"""

    rosflight_sim_dir = get_package_share_directory('rosflight_sim')
    param_file = os.path.join(rosflight_sim_dir, 'params', 'standalone_sim_params.yaml')

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

    dynamics_param_file_arg = DeclareLaunchArgument(
        "dynamics_param_file",
        default_value="",
        description="Parameter file that contains the dynamics of the vehicle, containing the vehicle mass parameter."
    )
    dynamics_param_file = LaunchConfiguration("dynamics_param_file")

    namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Namespace for this instance of the ROScopter autonomy stack"
    )
    namespace = LaunchConfiguration("namespace")

    # Start Rosflight SIL
    rosflight_sil_node = Node(
        package="rosflight_sim",
        executable="rosflight_sil_manager",
        name='rosflight_sil_manager',
        namespace=namespace,
        output="screen",
        parameters=[{"use_sim_time": use_sim_time, "use_timer": True}],
    )

    # Start standalone sensors
    standalone_sensor_node = Node(
        package="rosflight_sim",
        executable="standalone_sensors",
        name='standalone_sensors',
        namespace=namespace,
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}, dynamics_param_file],
    )

    # Start rosflight_io interface node
    bind_port_1 = LaunchConfiguration('bind_port_1')
    bind_port_1_arg = DeclareLaunchArgument(
        'bind_port_1',
        default_value="14520",
        description="Port to connect over UDP between ROSflightIO and SILBoard"
    )
    bind_port_2 = LaunchConfiguration('bind_port_2')
    bind_port_2_arg = DeclareLaunchArgument(
        'bind_port_2',
        default_value="14521",
        description="Port to connect over UDP between ROSflightIO and SILBoard"
    )
    rosflight_io_node = Node(
        package="rosflight_io",
        executable="rosflight_io",
        name='rosflight_io',
        namespace=namespace,
        output="screen",
        parameters=[{"udp": True,
                     "bind_port": bind_port_1,
                     "remote_port": bind_port_2,
                     "use_sim_time": use_sim_time}],
    )

    # Start sil_board
    sil_board_node = Node(
        package="rosflight_sim",
        executable="sil_board",
        name='sil_board',
        namespace=namespace,
        output="screen",
        parameters=[{"use_sim_time": use_sim_time,
                     "simulation_port": bind_port_2,
                     "ROS_port": bind_port_1,
                     }],
    )

    # Start rc_joy node for RC input
    rc_joy_node = Node(
        package="rosflight_sim",
        namespace=namespace,
        executable="rc.py",
        parameters=[{"use_vimfly": use_vimfly, "use_sim_time": use_sim_time}],
    )

    # Start viz transcriber node
    sim_aircraft_file = LaunchConfiguration('sim_aircraft_file')
    viz_transcriber_node = Node(
        package="rosflight_sim",
        executable="standalone_viz_transcriber",
        name='standalone_viz_transcriber',
        namespace=namespace,
        output="screen",
        parameters=[{"sim_aircraft_file": sim_aircraft_file}, param_file],
    )
    sim_aircraft_file_launch_arg = DeclareLaunchArgument(
        'sim_aircraft_file',
        default_value=TextSubstitution(text=os.path.join("common_resource", "multirotor.dae")),
        description="Path to the .dae file that defines the simulation mesh to visualize."
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            use_vimfly_arg,
            dynamics_param_file_arg,
            bind_port_1_arg,
            bind_port_2_arg,
            namespace_arg,
            rosflight_sil_node,
            sil_board_node,
            standalone_sensor_node,
            rosflight_io_node,
            rc_joy_node,
            viz_transcriber_node,
            sim_aircraft_file_launch_arg,
        ]
    )

