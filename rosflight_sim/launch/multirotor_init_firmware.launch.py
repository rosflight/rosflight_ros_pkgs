"""
File: fixedwing_init_firmware.launch.py
Author: Brandon Sutherland
Created: June 22, 2023
Last Modified: July 17, 2023
Description: ROS2 launch file used to load parameters and call services needed for initializing firmware for multirotors.
"""

import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import FindExecutable


def generate_launch_description():
    """Initialized rosflight firmware for flying a multirotor UAV in the sim"""

    # Call load parameter file service
    param_load_service_exec = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' service call ',
            '/param_load_from_file ',
            'rosflight_msgs/srv/ParamFile ',
            '"{filename: "' + os.path.join(
                get_package_share_directory('rosflight_sim'), 'params/multirotor_firmware/multirotor_combined.yaml"}'
            ) + '"'
        ]],
        shell=True
    )

    # Call calibrate IMU service
    imu_cal_service_exec = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' service call ',
            '/calibrate_imu ',
            'std_srvs/srv/Trigger '
        ]],
        shell=True
    )

    # Save params
    write_params_service_exec = ExecuteProcess(
        cmd=[[
            'sleep 10 ; ',
            FindExecutable(name='ros2'),
            ' service call ',
            '/param_write ',
            'std_srvs/srv/Trigger '
        ]],
        shell=True
    )

    return LaunchDescription([
        param_load_service_exec,
        imu_cal_service_exec,
        write_params_service_exec
    ])
