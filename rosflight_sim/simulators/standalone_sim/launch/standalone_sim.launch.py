from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    # Create the package directory
    rosflight_sim_share = FindPackageShare('rosflight_sim')

    rviz2_config_file = LaunchConfiguration('rviz2_config_file')
    rviz2_config_file_arg = DeclareLaunchArgument(
        'rviz2_config_file',
        default_value=PathJoinSubstitution([rosflight_sim_share, 'config', 'standalone_sim.rviz']),
        description="Path to the .rviz file that defines the RViz configuration."
    )
    rviz2_splash_file = PathJoinSubstitution(
        [rosflight_sim_share, 'standalone_resource', 'logo.png']
    )
    param_file = PathJoinSubstitution([rosflight_sim_share, 'params', 'standalone_sim_params.yaml'])

    sim_aircraft_file = LaunchConfiguration('sim_aircraft_file')
    sim_aircraft_file_launch_arg = DeclareLaunchArgument(
        'sim_aircraft_file',
        default_value="common_resource/multirotor.dae",
        description="Path to the .dae file that defines the simulation mesh to visualize."
    )

    return LaunchDescription(
        [
            rviz2_config_file_arg,
            sim_aircraft_file_launch_arg,
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=[
                    *["--x", "0"],
                    *["--y", "0"],
                    *["--z", "0"],
                    *["--yaw", "0"],
                    *["--pitch", "0"],
                    *["--roll", "3.1415926535"],
                    *["--frame-id", "world"],
                    *["--child-frame-id", "NED"],
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=[
                    *["--x", "0"],
                    *["--y", "0"],
                    *["--z", "0"],
                    *["--yaw", "-1.570796326"],
                    *["--pitch", "0"],
                    *["--roll", "3.1415926535"],
                    *["--frame-id", "aircraft_body"],
                    *["--child-frame-id", "stl_frame"],
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz2_config_file, "-s", rviz2_splash_file],
            ),
            Node(
                package="rosflight_sim",
                executable="standalone_viz_transcriber",
                name='standalone_viz_transcriber',
                output="screen",
                parameters=[{"sim_aircraft_file": sim_aircraft_file}, param_file],
            ),
        ]
    )
