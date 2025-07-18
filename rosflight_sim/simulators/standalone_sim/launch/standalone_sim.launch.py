import os
from launch import LaunchDescription
from launch.substitutions import TextSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Create the package directory
    rosflight_sim_share_dir = get_package_share_directory("rosflight_sim")

    rviz2_config_file = os.path.join(
        rosflight_sim_share_dir, "config", "standalone_sim.rviz"
    )
    rviz2_splash_file = os.path.join(rosflight_sim_share_dir, "standalone_resource", "logo.png")
    param_file = os.path.join(rosflight_sim_share_dir, 'params', 'standalone_sim_params.yaml')

    sim_aircraft_file = LaunchConfiguration('sim_aircraft_file')
    sim_aircraft_file_launch_arg = DeclareLaunchArgument(
        'sim_aircraft_file',
        default_value=TextSubstitution(text=os.path.join("common_resource", "multirotor.dae")),
        description="Path to the .dae file that defines the simulation mesh to visualize."
)

    return LaunchDescription(
        [
            sim_aircraft_file_launch_arg,
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=[
                    "--x",
                    "0",
                    "--y",
                    "0",
                    "--z",
                    "0",
                    "--yaw",
                    "0",
                    "--pitch",
                    "0",
                    "--roll",
                    "3.1415926535",
                    "--frame-id",
                    "world",
                    "--child-frame-id",
                    "NED",
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=[
                    "--x",
                    "0",
                    "--y",
                    "0",
                    "--z",
                    "0",
                    "--yaw",
                    "-1.570796326",
                    "--pitch",
                    "0",
                    "--roll",
                    "3.1415926535",
                    "--frame-id",
                    "aircraft_body",
                    "--child-frame-id",
                    "stl_frame",
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
                parameters=[{"sim_aircraft_file": sim_aircraft_file}, param_file]
            ),
        ]
    )
