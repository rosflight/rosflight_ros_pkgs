"""
File: fixedwing_init_firmware.launch.py
Author: Brandon Sutherland
Created: June 22, 2023
Last Modified: July 17, 2023
Description: ROS2 launch file used to load parameters and call services needed for initializing firmware for fixedwings.
"""

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch.substitutions import FindExecutable


def generate_launch_description():
    """Initialized rosflight firmware for flying a fixedwing UAV in the sim"""

    rosflight_sim = get_package_share_path('rosflight_sim')
    param_file = rosflight_sim / 'params' / 'fixedwing_firmware.yaml'

    # Call load parameter file service
    param_load_service_exec = ExecuteProcess(
        cmd=[
            FindExecutable(name='ros2'),
            'service',
            'call',
            '/param_load_from_file',
            'rosflight_msgs/srv/ParamFile',
            f'{{filename: {param_file}}}',
        ],
    )

    # Call calibrate IMU service
    imu_cal_service_exec = ExecuteProcess(
        cmd=[
            FindExecutable(name='ros2'),
            'service',
            'call',
            '/calibrate_imu',
            'std_srvs/srv/Trigger',
        ],
    )

    # Save params
    write_params_service_exec = TimerAction(
        period=10.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    FindExecutable(name='ros2'),
                    'service',
                    'call',
                    '/param_write',
                    'std_srvs/srv/Trigger',
                ],
            )
        ],
    )

    return LaunchDescription(
        [param_load_service_exec, imu_cal_service_exec, write_params_service_exec]
    )
