"""
Gazebo Harmonic launch for rosflight_sim.
Replaces gazebo_ros/gazebo.launch.py + spawn_entity.py with ros_gz_sim.
"""

import os
import sys
from pathlib import Path

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, ExecuteProcess, \
    TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution, FindExecutable
from launch_ros.actions import Node


def generate_launch_description():
    """This is a launch file that runs the bare minimum requirements fly a fixedwing in a gz Harmonic simulator"""

    # TODO: It would be better to use the launch argument configuration instead of manually parsing
    aircraft = 'anaconda'  # default aircraft
    # aircraft_3d_file = 'skyhunter'
    # aircraft_model = "skyhunter.dae'
    for arg in sys.argv:
        if arg.startswith("aircraft:="):
            aircraft = arg.split(":=")[1]

    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Whether the nodes will use sim time or not"
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    pkg_share = get_package_share_directory('rosflight_sim')
    world_file = os.path.join(pkg_share, 'gz_resource', 'runway.sdf')

    # Start simulator
    gz_args = LaunchConfiguration('gz_args')
    gz_args_launch_arg = DeclareLaunchArgument(
        'gz_args',
        default_value=TextSubstitution(text=world_file))

    resource_paths = [
        os.path.join(pkg_share, 'gz_resource'),
        os.path.join(pkg_share, 'common_resource'),
    ]
    resource_env = ':'.join(resource_paths)

    plugin_paths = [
        os.path.join(pkg_share, '..', '..', 'lib')  # install/rosflight_sim/lib
    ]
    plugin_env = ':'.join(plugin_paths)

    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': gz_args}.items(),
    )

    spawn_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory("rosflight_sim"),
                "launch/gz_sim_spawn.launch.py",
            )
        ]),
        launch_arguments={
            # 'robot_namespace': 'fixedwing',
            # 'paused' : False,
            'x': '0.0',
            'y': '0.0',
            'z': '0.2',
            'yaw': '4.71',
        }.items()
    )

    # aircraft_model = LaunchConfiguration("aircraft_model")
    """This is a launch file that runs the bare minimum requirements fly a fixedwing in a standalone simulator"""
    dynamics_file = os.path.join(pkg_share, 'params', f'{aircraft}_dynamics.yaml')
    # dynamics_file = os.path.join(get_package_share_directory('rosflight_sim'), 'params', 'anaconda_dynamics.yaml')
    dynamics_param_file_arg = DeclareLaunchArgument(
        "dynamics_param_file",
        default_value=dynamics_file,
        # default_value=os.path.join(get_package_share_directory('ros_gz_state_viz'), 'params', 'chgb_dynamics.yaml'),
        description="Parameter file that contains the dynamics of the vehicle, containing the vehicle mass parameter."
    )
    dynamics_param_file = LaunchConfiguration("dynamics_param_file")

    # Start independent nodes
    independent_nodes_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory("rosflight_sim"),
                "launch", "common_nodes_gz_sim.launch.py",
            )
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )

    # Start forces and moments
    fw_forces_moments_node = Node(
        package="rosflight_sim",
        executable="fixedwing_forces_and_moments",
        name="fixedwing_forces_and_moments",
        output="screen",
        parameters=[
            os.path.join(get_package_share_directory('rosflight_sim'),
                         f'params/{aircraft}_dynamics.yaml'),
            {"use_sim_time": use_sim_time},
        ],
    )

    control_type = "default"
    rosplane_pkg_share = get_package_share_directory('rosplane')
    autopilot_params = os.path.join(
        rosplane_pkg_share,
        'params',
        aircraft + '_autopilot_params.yaml'
    )

    estimator_params = os.path.join(
        rosplane_pkg_share,
        'params',
        'estimator.yaml'
    )

    # Start rosplane
    rosplane_nodes_include = LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Whether or not to use the /clock topic in simulation to run timers."
        ),
        DeclareLaunchArgument(
            "state_topic_remap",
            default_value='estimated_state',
            description="Topic that controller and path planners will listen to for state information. Defaults to the topic that the estimator publishes to"
        ),
        DeclareLaunchArgument(
            'command_publisher_remap',
            default_value='/command',
        ),
        DeclareLaunchArgument(
            'controller_command_publisher_remap',
            default_value='/controller_command',
        ),
        Node(
            package='rosplane',
            executable='controller',
            name='controller',
            parameters=[
                autopilot_params,
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
            output='screen',
            arguments=[control_type],
            remappings=[
                ('/command', LaunchConfiguration('command_publisher_remap')),
                ('/estimated_state', LaunchConfiguration('state_topic_remap')),
            ]
        ),
        Node(
            package='rosplane',
            executable='path_follower',
            name='path_follower',
            parameters=[
                autopilot_params,
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
            remappings=[
                ('/controller_command', LaunchConfiguration('controller_command_publisher_remap')),
                ('/estimated_state', LaunchConfiguration('state_topic_remap')),
            ]
        ),
        Node(
            package='rosplane',
            executable='path_manager',
            name='path_manager',
            parameters=[
                autopilot_params,
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
            remappings=[
                ('/estimated_state', LaunchConfiguration('state_topic_remap')),
            ]
        ),
        Node(
            package='rosplane',
            executable='path_planner',
            name='path_planner',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time'),
                 'num_waypoints_to_publish_at_start': 10},
            ],
            remappings=[
                ('/estimated_state', LaunchConfiguration('state_topic_remap')),
            ]
        ),
        Node(
            package='rosplane',
            executable='estimator',
            name='estimator',
            output='screen',
            parameters=[
                estimator_params,
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
        )
    ])

    unpause_sim_exec = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'gz', 'service',
                    '-s', '/world/default/control',
                    '--reqtype', 'gz.msgs.WorldControl',
                    '--reptype', 'gz.msgs.Boolean',
                    '--timeout', '3000',
                    '--req', 'pause: false'
                ],
                output='screen'
            )
        ]
    )

    # Initialize firmware nodes
    # init_firmware_nodes = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         os.path.join(
    #             get_package_share_directory("rosflight_sim"),
    #             "launch", "fixedwing_init_firmware.launch.py",
    #         )
    #     ]),
    #     launch_arguments={
    #         'use_sim_time': use_sim_time
    #     }.items()
    # )

    # Call load parameter file service
    param_load_service_exec = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' service call ',
            '/param_load_from_file ',
            'rosflight_msgs/srv/ParamFile ',
            '"{filename: "' + os.path.join(
                get_package_share_directory('rosflight_sim'), 'params/fixedwing_firmware.yaml"}'
            ) + '"'
        ]],
        shell=True
    )

    # Call calibrate IMU service
    imu_cal_service_exec = ExecuteProcess(
        cmd=[[
            'sleep 2; ',
            FindExecutable(name='ros2'),
            ' service call ',
            '/calibrate_imu ',
            'std_srvs/srv/Trigger '
        ]],
        shell=True
    )

    # Call calibrate barometer service
    baro_cal_service_exec = ExecuteProcess(
        cmd=[[
            'sleep 2; ',
            FindExecutable(name='ros2'),
            ' service call ',
            '/calibrate_baro ',
            'std_srvs/srv/Trigger '
        ]],
        shell=True
    )

    # Arm vehicle service
    arm_vehicle_service_exec = ExecuteProcess(
        cmd=[[
            'sleep 3; ',
            FindExecutable(name='ros2'),
            ' service call ',
            '/toggle_arm ',
            'std_srvs/srv/Trigger '
        ]],
        shell=True
    )

    # RC override service
    rc_override_service_exec = ExecuteProcess(
        cmd=[[
            'sleep 8; ',
            FindExecutable(name='ros2'),
            ' service call ',
            '/toggle_override ',
            'std_srvs/srv/Trigger '
        ]],
        shell=True
    )

    # Save params
    write_params_service_exec = ExecuteProcess(
        cmd=[[
            'sleep 10; ',
            FindExecutable(name='ros2'),
            ' service call ',
            '/param_write ',
            'std_srvs/srv/Trigger '
        ]],
        shell=True
    )

    # Call calibrate IMU service
    mission_file = os.path.join(pkg_share, 'missions', 'gz_sim_fixedwing_mission.yaml')
    waypoint_definition_service_exec = ExecuteProcess(
        cmd=[[
            'sleep 20; ',
            FindExecutable(name='ros2'),
            ' service call ',
            '/load_mission_from_file ',
            'rosflight_msgs/srv/ParamFile \"{filename: ' + mission_file + '}\" '
        ]],
        shell=True
    )

    return LaunchDescription([
        use_sim_time_arg,
        dynamics_param_file_arg,
        gz_args_launch_arg,
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_env),
        SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', plugin_env),
        gz_sim_launch,
        spawn_robot_launch,
        rosplane_nodes_include,
        independent_nodes_include,
        fw_forces_moments_node,
        unpause_sim_exec,
        # init_firmware_nodes,
        param_load_service_exec,
        imu_cal_service_exec,
        baro_cal_service_exec,
        # write_params_service_exec,
        arm_vehicle_service_exec,
        rc_override_service_exec,
        waypoint_definition_service_exec
    ])
