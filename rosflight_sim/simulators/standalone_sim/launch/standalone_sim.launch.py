import os
from launch import LaunchDescription
from launch.substitutions import TextSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition


def generate_launch_description():
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Whether the nodes will use sim time or not"
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Create the package directory
    rosflight_sim_share_dir = get_package_share_directory("rosflight_sim")

    rviz2_config_file = LaunchConfiguration('rviz2_config_file')
    rviz2_config_file_arg = DeclareLaunchArgument(
        'rviz2_config_file',
        default_value=TextSubstitution(text=os.path.join(rosflight_sim_share_dir, "config", "standalone_sim.rviz")),
        description="Path to the .rviz file that defines the RViz configuration."
    )
    rviz2_splash_file = os.path.join(rosflight_sim_share_dir, "standalone_resource", "logo.png")
    param_file = os.path.join(rosflight_sim_share_dir, 'params', 'standalone_sim_params.yaml')

    return LaunchDescription(
        [
            use_sim_time_arg,
            rviz2_config_file_arg,
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
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz2_config_file, "-s", rviz2_splash_file],
            ),
            # Start time manager, if applicable
            Node(
                package="rosflight_sim",
                executable="standalone_time_manager",
                name='standalone_time_manager',
                output="screen",
                condition=IfCondition(use_sim_time),
                parameters=[param_file]
            ),
        ]
    )
