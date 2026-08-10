"""
File: fixedwing_init_firmware.launch.py
Author: Brandon Sutherland
Created: June 22, 2023
Last Modified: July 17, 2023
Description: ROS2 launch file used to load parameters and call services needed for initializing firmware for fixedwings.
"""

import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import FindExecutable, LaunchConfiguration


def evaluate_namespace_arg(context, *args, **kwargs):
    # 'context' is the key here. It can evaluate substitutions.
    # 'my_arg_name' should match the name declared in DeclareLaunchArgument
    namespace = LaunchConfiguration('namespace')
    namespace_string = context.perform_substitution(namespace)

    # Now you can use 'arg_value_string' as a normal Python string
    #print(f"The evaluated argument value is: {namespace_string}")

    param_load_from_file="/param_load_from_file "
    calibrate_imu="/calibrate_imu "
    param_write="/param_write "
    if (namespace_string != ''):
        param_load_from_file=f"/{namespace_string}/param_load_from_file "
        calibrate_imu=f"/{namespace_string}/calibrate_imu "
        param_write=f"/{namespace_string}/param_write "
    
    # You must return a list of launch actions (or None)
    return [
        # Call load parameter file service
        ExecuteProcess(
            cmd=[[
                FindExecutable(name='ros2'),
                ' service call ',
                param_load_from_file,
                'rosflight_msgs/srv/ParamFile ',
                '"{filename: "' + os.path.join(
                    get_package_share_directory('rosflight_sim'), 'params/fixedwing_firmware.yaml"}'
                ) + '"'
            ]],
            shell=True
        ),

        # Call calibrate IMU service
        ExecuteProcess(
            cmd=[[
                FindExecutable(name='ros2'),
                ' service call ',
                calibrate_imu,
                'std_srvs/srv/Trigger '
            ]],
            shell=True
        ),

        # Save params
        ExecuteProcess(
            cmd=[[
                'sleep 10 ; ',
                FindExecutable(name='ros2'),
                ' service call ',
                param_write,
                'std_srvs/srv/Trigger '
            ]],
            shell=True
        ),
    ]

def generate_launch_description():
    """Initialized rosflight firmware for flying a fixedwing UAV in the sim"""

    namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value='',
        description="Namespace for the simulation"
    )

    return LaunchDescription([
        namespace_arg,
        OpaqueFunction(function=evaluate_namespace_arg),
    ])
