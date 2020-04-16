# This is a launch file that runs the bare minimum requirements fly a fixedwing in gazebo

import os
import sys

argv = sys.argv[1:]

import launch
import launch.actions
import launch.substitutions
import launch_ros.actions as lr_actions

from launch import LaunchDescription
# from launch_ros.actions import Node

launch.actions.IncludeLaunchDescription("$(find rosflight_sim)/launch/fixedwing.launch")

# rosflight_io node
rf_io_node = lr_actions.Node(
            package='rosflight',
            node_namespace='turtlesim1',
            node_executable='turtlesim_node',
            node_name='rosflight_io'
            )
#

# safety_pilot node
sp_node = lr_actions.Node(
        package='rosflight_utils',
        node_namespace='turtlesim1',
        node_executable='turtlesim_node',
        node_name='safety_pilot'
)



def generate_launch_description():
    return LaunchDescription([
        rf_io_node,
        sp_node,
        Node(
            package='turtlesim',
            node_executable='mimic',
            node_name='mimic',
            remappings=[
                ('/input/pose', '/turtlesim1/turtle1/pose'),
                ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
            ]
        )
    ]) # LaunchDescription
#
