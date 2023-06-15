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
        namespace='scout_mini',
        parameters=[ekf_config, {
            'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [slam_toolbox_pkg, '/launch/online_async_launch.py']),
    )

    zed2_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('zed_wrapper'), 
                'launch', 'zed2.launch.py'
            ])
        ]),
        launch_arguments={
            'publish_tf' : 'false',
            'camera_name' : 'scout_mini',
            'cam_pose' : '0.277812 0 0.176212 0 0 0'
        }
    )
    ld.add_action(slam_params_file)
    #ld.add_action(slam_toolbox_launch)
    ld.add_action(sim_time_argument)
    ld.add_action(robot_localization_node)

    return ld
