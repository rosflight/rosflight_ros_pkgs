import os

from ament_index_python import get_package_share_directory, get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import TextSubstitution, LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Launches a multirotor SIL vehicle in Gazebo"""

    # Launch Arguments
    color = LaunchConfiguration('color')
    color_launch_arg = DeclareLaunchArgument(
        'color', default_value=TextSubstitution(text='White')
    )
    x = LaunchConfiguration('x')
    x_launch_arg = DeclareLaunchArgument(
        'x', default_value=TextSubstitution(text='0')
    )
    y = LaunchConfiguration('y')
    y_launch_arg = DeclareLaunchArgument(
        'y', default_value=TextSubstitution(text='0')
    )
    z = LaunchConfiguration('z')
    z_launch_arg = DeclareLaunchArgument(
        'z', default_value=TextSubstitution(text='0.1')
    )
    yaw = LaunchConfiguration('yaw')
    yaw_launch_arg = DeclareLaunchArgument(
        'yaw', default_value=TextSubstitution(text='0')
    )
    paused = LaunchConfiguration('paused')
    paused_launch_arg = DeclareLaunchArgument(
        'paused', default_value=TextSubstitution(text='false')
    )
    gui = LaunchConfiguration('gui')
    gui_launch_arg = DeclareLaunchArgument(
        'gui', default_value=TextSubstitution(text='true')
    )
    verbose = LaunchConfiguration('verbose')
    verbose_launch_arg = DeclareLaunchArgument(
        'verbose', default_value=TextSubstitution(text='false')
    )
    debug = LaunchConfiguration('verbose')
    debug_launch_arg = DeclareLaunchArgument(
        'debug', default_value=TextSubstitution(text='false')
    )
    world_file = LaunchConfiguration('world_file')
    world_file_launch_arg = DeclareLaunchArgument(
        'world_file', default_value=TextSubstitution(text='worlds/empty.world')
    )
    tf_prefix = LaunchConfiguration('tf_prefix')
    tf_prefix_launch_argument = DeclareLaunchArgument(
        'tf_prefix', default_value=TextSubstitution(text="")
    )
    enable_logging = LaunchConfiguration('enable_logging')
    enable_logging_launch_argument = DeclareLaunchArgument(
        'enable_logging', default_value=TextSubstitution(text="false")
    )
    enable_ground_truth = LaunchConfiguration('enable_ground_truth')
    enable_ground_truth_launch_argument = DeclareLaunchArgument(
        'enable_ground_truth', default_value=TextSubstitution(text="true")
    )
    log_file = LaunchConfiguration('log_file')
    log_file_launch_argument = DeclareLaunchArgument(
        'log_file', default_value=TextSubstitution(text="multirotor")
    )
    enable_wind = LaunchConfiguration('enable_wind')
    enable_wind_launch_argument = DeclareLaunchArgument(
        'enable_wind', default_value=TextSubstitution(text="true")
    )
    robot_namespace = LaunchConfiguration('robot_namespace')
    robot_namespace_launch_argument = DeclareLaunchArgument(
        'robot_namespace', default_value=TextSubstitution(text="multirotor")
    )
    gazebo_namespace = LaunchConfiguration('gazebo_namespace')
    gazebo_namespace_launch_argument = DeclareLaunchArgument(
        'gazebo_namespace', default_value=TextSubstitution(text="")
    )

    # Start simulator
    gazebo_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch/gazebo.launch.py'
            )
        ),
        launch_arguments={
            'paused': paused,
            'gui': gui,
            'verbose': verbose,
            'debug': debug,
            'world_name': world_file
        }.items()
    )

    # Spawn vehicle
    spawn_vehicle_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        respawn=False,
        output='screen',
        parameters=[
            os.path.join(get_package_share_directory('rosflight_sim'), 'params/multirotor.yaml'),
            {
                'tf_prefix': tf_prefix
            }
        ],
        arguments=[
            '-file', os.path.join(get_package_share_directory('rosflight_sim'), 'xacro/multirotor.urdf'),
            '-entity', 'multirotor',
            '-robot_namespace', robot_namespace,
            '-x', x,
            '-y', y,
            '-z', z,
            '-Y', yaw,
            '--ros-args', '--log-level', 'debug'
        ]
    )

    return LaunchDescription([
        color_launch_arg,
        x_launch_arg,
        y_launch_arg,
        z_launch_arg,
        yaw_launch_arg,
        paused_launch_arg,
        gui_launch_arg,
        verbose_launch_arg,
        debug_launch_arg,
        world_file_launch_arg,
        tf_prefix_launch_argument,
        enable_logging_launch_argument,
        enable_ground_truth_launch_argument,
        log_file_launch_argument,
        enable_wind_launch_argument,
        robot_namespace_launch_argument,
        gazebo_namespace_launch_argument,
        gazebo_launch_include,
        spawn_vehicle_node
    ])
