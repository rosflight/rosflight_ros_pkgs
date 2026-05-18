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
    """This is a launch file that runs the bare minimum requirements fly a multirotor in a gz Harmonic simulator"""

    aircraft = 'multirotor'
    aircraft_3d_file = 'multirotor'
    # aircraft_3d_file = 'anaconda'
    for arg in sys.argv:
        if arg.startswith('aircraft:='):
            aircraft = arg.split(':=')[1]
    if aircraft == 'multirotor':
        aircraft_3d_file = 'multirotor'


    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Whether the nodes will use sim time or not"
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    pkg_share = get_package_share_directory('rosflight_sim')

    dynamics_file = os.path.join(pkg_share, 'params', f'{aircraft}_dynamics.yaml')
    dynamics_param_file_arg = DeclareLaunchArgument(
        "dynamics_param_file",
        default_value=dynamics_file,
        description="Parameter file that contains the dynamics of the vehicle, containing the vehicle mass parameter."
    )

    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')
    gz_args = LaunchConfiguration('gz_args')

    x_launch_arg = DeclareLaunchArgument('x', default_value=TextSubstitution(text='0'))
    y_launch_arg = DeclareLaunchArgument('y', default_value=TextSubstitution(text='0'))
    z_launch_arg = DeclareLaunchArgument('z', default_value=TextSubstitution(text='0.2'))
    yaw_launch_arg = DeclareLaunchArgument('yaw', default_value=TextSubstitution(text='4.71'))
    gz_args_launch_arg = DeclareLaunchArgument(
        'gz_args',
        default_value=TextSubstitution(text=os.path.join(pkg_share, 'gz_resource', 'runway.sdf')))

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

    xacro_filepath_string = os.path.join(pkg_share, f'xacro/{aircraft}.urdf.xacro')
    urdf_filepath_string = os.path.join(pkg_share, f'gz_resource/{aircraft}.urdf')
    robot_description = xacro.process_file(
        xacro_filepath_string,
        mappings={
            'mesh_file_location': os.path.join(pkg_share, f'common_resource/{aircraft_3d_file}.dae')
        }
    ).toxml()
    Path(urdf_filepath_string).write_text(robot_description)

    spawn_vehicle_node = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-world', 'default',
            '-name', 'robot',
            '-file', urdf_filepath_string,
            '-x', x,
            '-y', y,
            '-z', z,
            '-Y', yaw,
        ]
    )

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
    mr_forces_moments_node = Node(
        package="rosflight_sim",
        executable="multirotor_forces_and_moments",
        name="multirotor_forces_and_moments",
        output="screen",
        parameters=[
            os.path.join(get_package_share_directory('rosflight_sim'),
                         f'params/multirotor_dynamics.yaml'),
            {"use_sim_time": use_sim_time},
        ],
    )

    # Start roscopter
    roscopter_dir = get_package_share_directory('roscopter')
    controller_param_file = os.path.join(roscopter_dir, 'params', 'multirotor.yaml')
    estimator_param_file = os.path.join(roscopter_dir, 'params', 'estimator.yaml')

    hotstart_estimator_arg = DeclareLaunchArgument(
        "hotstart_estimator",
        default_value="false",
        description="Whether the estimator will hotstart based on the contents of 'params/hotstart'"
    )
    hotstart_estimator = LaunchConfiguration('hotstart_estimator')

    state_remap_arg = DeclareLaunchArgument(
        "state_topic",
        default_value="estimated_state",
        description="Topic name the every node but the estimator will listen to for state information."
    )
    state_remap = LaunchConfiguration('state_topic')

    roscopter_nodes_include = LaunchDescription([
        state_remap_arg,
        hotstart_estimator_arg,
        Node(
            package='roscopter',
            executable='controller',
            name='controller',
            output='screen',
            parameters=[controller_param_file],
            remappings=[('estimated_state', state_remap)]
        ),
        Node(
            package='roscopter',
            executable='estimator',
            name='estimator',
            output='screen',
            parameters=[estimator_param_file, {"hotstart_estimator": hotstart_estimator}],
        ),
        Node(
            package='roscopter',
            executable='trajectory_follower',
            name='trajectory_follower',
            output='screen',
            parameters=[controller_param_file],
            remappings=[('estimated_state', state_remap)]
        ),
        Node(
            package='roscopter',
            executable='path_manager',
            name='path_manager',
            output='screen',
            parameters=[controller_param_file],
            remappings=[('estimated_state', state_remap)]
        ),
        Node(
            package='roscopter',
            executable='path_planner',
            name='path_planner',
            output='screen',
            parameters=[controller_param_file],
            remappings=[('estimated_state', state_remap)]
        ),
        # Node(
        #     package='roscopter',
        #     executable='ext_att_transcriber',
        #     name='external_attitude_transcriber',
        #     output='screen',
        #     remappings=[('estimated_state', state_remap)]
        # ),
    ])

    unpause_sim_exec = TimerAction(
        period=0.0,
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
    #             "launch", "multirotor_init_firmware.launch.py",
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
                get_package_share_directory('rosflight_sim'), 'params/multirotor_firmware/multirotor_combined.yaml"}'
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
    mission_file = os.path.join(pkg_share, 'missions', 'gz_sim_multirotor_mission.yaml')
    waypoint_definition_service_exec = ExecuteProcess(
        cmd=[[
            'sleep 20; ',
            FindExecutable(name='ros2'),
            ' service call ',
            '/path_planner/load_mission_from_file ',
            'rosflight_msgs/srv/ParamFile \"{filename: ' + mission_file + '}\" '
        ]],
        shell=True
    )


    return LaunchDescription([
        use_sim_time_arg,
        dynamics_param_file_arg,
        x_launch_arg,
        y_launch_arg,
        z_launch_arg,
        yaw_launch_arg,
        gz_args_launch_arg,
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_env),
        SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', plugin_env),
        gz_sim_launch,
        spawn_vehicle_node,
        roscopter_nodes_include,
        independent_nodes_include,
        mr_forces_moments_node,
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
