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
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')

    

    scout_mini_control_pkg = get_package_share_directory('scout_mini_control')
    slam_toolbox_pkg = get_package_share_directory('slam_toolbox')

    slam_params_file = DeclareLaunchArgument(
        'slam_params_file', default_value=str(os.path.join(scout_mini_control_pkg, 'config', 'slam.yaml')))
    
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(get_package_share_directory("scout_mini_control"),
                                   'config', 'slam.yaml'),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock')
    
    declare_namespace_argument = DeclareLaunchArgument('namespace', default_value='scout_mini',
        description='Top-level namespace')

    ld = LaunchDescription()

    depthimage_to_laserscan_launch = Node(
        package='depthimage_to_laserscan', executable='depthimage_to_laserscan_node',
        remappings=[('depth', '/scout_mini/zed_depth_camera/depth/image_raw'),
                    ('depth_camera_info', '/scout_mini/zed_depth_camera/depth/camera_info'),
                    ('scan', 'scan')],
        parameters=[{
                'range_min' : 0.2,
                'range_max' : 20.0,
                'scan_time' : 0.0333333,
                'output_frame' : 'zed2_right_camera_frame',
        }],
        name='depthimage_to_laserscan'
    )


    # slam_toolbox_node = Node(
    #     package='slam_toolbox',
    #     executable='async_slam_toolbox_node',
    #     name='slam_toolbox',
    #     namespace='scout_mini',
    #     output='screen',
    #     parameters=[
    #         {'slam_params_file' : str(os.path.join(get_package_share_directory("scout_mini_control"),
    #                                'config', 'slam.yaml'))},
    #         {'use_sim_time' : use_sim_time}
    #     ]
    # )

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [slam_toolbox_pkg, '/launch/online_async_launch.py']),
    )
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch', 'navigation_launch.py',
            ])
        ]),
        launch_arguments={
            'namespace': 'scout_mini',
            'use_sim_time' : 'true',
            'params_file' : str(os.path.join(scout_mini_control_pkg, 'config', 'nav2.yaml')),
            # 'container_name' : 'scout_mini',

        }.items(),
    )
    # Launch Arguments
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_namespace_argument)
    ld.add_action(slam_params_file)
    # Navigation Nodes
    #ld.add_action(slam_toolbox_launch)
    ld.add_action(slam_toolbox_launch)
    #ld.add_action(nav2_bringup_launch)
    ld.add_action(depthimage_to_laserscan_launch)


    return ld
