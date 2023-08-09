import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    """This is a launch file that runs the bare minimum requirements fly a multirotor in gazebo"""

    # Start simulator
    simulator_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rosflight_sim'),
                'launch/multirotor.launch.py'
            )
        )
    )

    # Start rosflight_io interface node
    rosflight_io_node = Node(
        package='rosflight',
        executable='rosflight_io',
        output='screen',
        parameters=[
            {'udp': True}
        ]
    )

    # Start rc_joy node for RC input
    rc_joy_node = Node(
        package='rosflight_utils',
        executable='rc_joy.py',
        remappings=[
            ('/RC', '/multirotor/RC')
        ]
    )

    return LaunchDescription([
        simulator_launch_include,
        rosflight_io_node,
        rc_joy_node
    ])
