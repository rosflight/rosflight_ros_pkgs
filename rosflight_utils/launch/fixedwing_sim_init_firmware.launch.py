import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import FindExecutable
from launch.actions import ExecuteProcess


def generate_launch_description():
    """Set up an uninitialized firmware for flying a fixedwing UAV"""

    # Call load parameter file service
    param_load_service_exec = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' service call ',
            '/param_load_from_file ',
            'rosflight_msgs/srv/ParamFile ',
            '"{filename: "' + os.path.join(
                get_package_share_directory('rosflight_utils'), 'params/fixedwing_firmware.yaml"}'
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

    # Call calibrate airspeed service
    airspeed_cal_service_exec = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' service call ',
            '/calibrate_airspeed ',
            'std_srvs/srv/Trigger'
        ]],
        shell=True
    )

    return LaunchDescription([
        param_load_service_exec,
        imu_cal_service_exec,
        airspeed_cal_service_exec
    ])
