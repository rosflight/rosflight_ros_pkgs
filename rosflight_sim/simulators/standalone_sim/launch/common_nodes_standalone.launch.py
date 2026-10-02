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
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
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

    use_firmware_timer_arg = DeclareLaunchArgument(
        "use_firmware_timer", default_value="true",
        description="Run the firmware from its own ROS timer"
    )
    use_time_manager_arg = DeclareLaunchArgument(
        "use_time_manager", default_value=use_sim_time,
        description="Start the standalone wall-driven simulation clock"
    )
    imu_frame_id_arg = DeclareLaunchArgument(
        "imu_frame_id", default_value="",
        description="Frame ID for the simulated IMU topic"
    )
    imu_update_frequency_arg = DeclareLaunchArgument(
        "imu_update_frequency", default_value="400.0",
        description="Simulated IMU update frequency in Hz"
    )
    clock_sync_frequency_arg = DeclareLaunchArgument(
        "clock_sync_frequency", default_value="0.0",
        description="Clock acknowledgment rate for lockstep simulation; zero disables it"
    )
    rosflight_io_frame_id_arg = DeclareLaunchArgument(
        "rosflight_io_frame_id", default_value="world",
        description="Frame ID for ROSflight IO IMU messages"
    )

    # Start Rosflight SIL
    rosflight_sil_node = Node(
        package="rosflight_sim",
        executable="rosflight_sil_manager",
        name='rosflight_sil_manager',
        output="screen",
        parameters=[{"use_sim_time": use_sim_time,
                     "use_timer": LaunchConfiguration("use_firmware_timer")}],
    )

    # Start sil_board
    sil_board_node = Node(
        package="rosflight_sim",
        executable="sil_board",
        name='sil_board',
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # Start standalone sensors
    standalone_sensor_node = Node(
        package="rosflight_sim",
        executable="standalone_sensors",
        name='standalone_sensors',
        output="screen",
        parameters=[{"use_sim_time": use_sim_time,
                     "imu_frame_id": LaunchConfiguration("imu_frame_id"),
                     "imu_update_frequency": LaunchConfiguration("imu_update_frequency"),
                     "clock_sync_frequency": LaunchConfiguration("clock_sync_frequency")},
                    dynamics_param_file],
    )

    # Start rosflight_io interface node
    rosflight_io_node = Node(
        package="rosflight_io",
        executable="rosflight_io",
        name='rosflight_io',
        output="screen",
        parameters=[{"udp": True,
                     "use_sim_time": use_sim_time,
                     "frame_id": LaunchConfiguration("rosflight_io_frame_id")}],
    )

    # Start rc_joy node for RC input
    rc_joy_node = Node(
        package="rosflight_sim",
        executable="rc.py",
        parameters=[{"use_vimfly": use_vimfly, "use_sim_time": use_sim_time}],
    )

    # Start time manager, if applicable
    time_manager_node = Node(
        package="rosflight_sim",
        executable="standalone_time_manager",
        name='standalone_time_manager',
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_time_manager")),
        parameters=[param_file]
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            use_vimfly_arg,
            dynamics_param_file_arg,
            use_firmware_timer_arg,
            use_time_manager_arg,
            imu_frame_id_arg,
            imu_update_frequency_arg,
            clock_sync_frequency_arg,
            rosflight_io_frame_id_arg,
            rosflight_sil_node,
            sil_board_node,
            standalone_sensor_node,
            rosflight_io_node,
            rc_joy_node,
            time_manager_node,
        ]
    )
