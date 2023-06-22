#!/usr/bin/env python3

import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration

from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess
)


def generate_launch_description():

    ld = LaunchDescription()

    scout_mini_control_pkg = get_package_share_directory('scout_mini_control')

    sim_time_argument = DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                              description='Flag to enable use_sim_time')

    nav2_bringup_launch = ExecuteProcess(
        name="launch_navigation",
        cmd=[
            "ros2",
             "launch",
            PathJoinSubstitution(
                [
                    FindPackageShare("nav2_bringup"),
                    "launch",
                    "navigation_launch.py",
                ]
            ),
            "use_sim_time:=False",
            str("params_file:=" +
                str(os.path.join(scout_mini_control_pkg, 'config', 'nav2.yaml')))
        ],
        output="screen"

    )
    depthimage_to_laserscan_launch = Node(
        package='depthimage_to_laserscan', executable='depthimage_to_laserscan_node',
        remappings=[('depth', '/scout_mini/zed_depth_camera/depth/image_raw'),
                    ('depth_camera_info', '/scout_mini/zed_depth_camera/depth/camera_info'),
                    ('scan', 'scout_mini/scan')],
        parameters=[{
                'range_min' : 0.2,
                'range_max' : 20.0,
                'scan_time' : 0.0333333,
                'output_frame' : 'zed2_right_camera_frame',
        }],
        name='depthimage_to_laserscan'
    )

    nav2_bringup_launch = ExecuteProcess(
        name="launch_navigation",
        cmd=[
            "ros2",
            "launch",
            PathJoinSubstitution(
                [
                    FindPackageShare("nav2_bringup"),
                    "launch",
                    "navigation_launch.py",
                ]
            ),
            "use_sim_time:=True",
            str("params_file:=" + str(os.path.join(scout_mini_control_pkg, 'config', 'nav2.yaml')))
        ],
        output="screen"

    )

    # Navigation Nodes
    ld.add_action(nav2_bringup_launch)
    ld.add_action(depthimage_to_laserscan_launch)

    ld.add_action(sim_time_argument)

    return ld
