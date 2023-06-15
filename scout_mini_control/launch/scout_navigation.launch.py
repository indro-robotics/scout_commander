#!/usr/bin/env python3

import os
import xacro

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    scout_mini_control_pkg = get_package_share_directory(
        'scout_mini_control')
    ekf_config = os.path.join(scout_mini_control_pkg, 'config/ekf.yaml')
    ld = LaunchDescription()
    slam_toolbox_pkg = get_package_share_directory('slam_toolbox')

    sim_time_argument = DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                              description='Flag to enable use_sim_time')

    slam_params_file = DeclareLaunchArgument(
        'slam_params_file', default_value=str(os.path.join(scout_mini_control_pkg, 'config', 'slam_toolbox.yaml')))

    ld.add_action(slam_params_file)

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {
            'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [slam_toolbox_pkg, '/launch/online_async_launch.py']),
    )

    ld.add_action(slam_params_file)
    ld.add_action(slam_toolbox_launch)
    ld.add_action(sim_time_argument)
    ld.add_action(robot_localization_node)

    return ld
