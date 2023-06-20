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

    scanner_arg = DeclareLaunchArgument(
        name='scanner', default_value='scout_mini',
        description='Namespace for sample topics'
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
            str("params_file:=" +
                str(os.path.join(scout_mini_control_pkg, 'config', 'nav2.yaml')))
        ],
        output="screen"

    )
    pointcloud_to_laserscan_launch = Node(
        package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
        remappings=[('cloud_in', [LaunchConfiguration(variable_name='scanner'), '/zed_depth_camera/points']),
                    ('scan', [LaunchConfiguration(variable_name='scanner'), '/scan'])],
        parameters=[{
                'target_frame': 'cloud',
                'transform_tolerance': 0.01,
                'min_height': 0.0,
                'max_height': 1.0,
                'angle_min': -1.0472,  # -M_PI/2
                'angle_max': 1.0472,  # M_PI/2
                'angle_increment': 0.0087,  # M_PI/360.0
                'scan_time': 0.03333,
                'range_min': 0.2,
                'range_max': 20.0,
                'use_inf': True,
                'inf_epsilon': 1.0
        }],
        name='pointcloud_to_laserscan'
    )

    # Navigation Nodes
    # ld.add_action(nav2_bringup_launch)
    ld.add_action(scanner_arg)
    ld.add_action(pointcloud_to_laserscan_launch)
    ld.add_action(sim_time_argument)

    return ld
