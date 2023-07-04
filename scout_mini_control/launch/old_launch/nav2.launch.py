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
from launch.conditions import IfCondition, UnlessCondition, LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration

from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess
)


def generate_launch_description():

    # Package Share Directories
    scout_mini_control_pkg = get_package_share_directory('scout_mini_control')

    # Launch Configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')
    slam_params_file = LaunchConfiguration('slam_params_file')
    declare_mode_argument = LaunchConfiguration('mode')

    #Declaring Arguments
    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation/Gazebo clock')
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(get_package_share_directory("scout_mini_control"),
                                   'config', 'slam.yaml'),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')
    
    declare_mode_argument = DeclareLaunchArgument(
        'mode',
        default_value='mapping',
        description='The navigation mode that the robot will be launched in: values are "mapping" and "localization"'
    )
    declare_map_file = DeclareLaunchArgument(
        'map',
        default_value='map.yaml',
        description='The map file to be used in the localization mode of navigation'

    )
    

    # Assigning Variables
    map_file = os.path.join(get_package_share_directory(
        'scout_mini_control'), 'maps', str(LaunchConfiguration('map'))
    )

    # Launching Nodes
    start_async_slam_toolbox_node = Node(
        parameters=[
          slam_params_file,
          {'use_sim_time': use_sim_time}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='scout_mini',
        output='screen',
        condition=LaunchConfigurationEquals('mode', 'mapping'))

    depthimage_to_laserscan_launch = Node(
        package='depthimage_to_laserscan', executable='depthimage_to_laserscan_node',
        remappings=[('depth', '/scout_mini/zed_node/depth/depth_registered'),
                    ('depth_camera_info', '/scout_mini/zed_node/depth/camera_info'),
                    ('scan', 'scout_mini/scan')],
        parameters=[{
                'range_min' : 0.2,
                'scan_height' : 5,
                'range_max' : 10.0,
                'scan_time' : 0.0333333,
                'output_frame' : 'scout_mini_right_camera_frame',
        }],
        name='depthimage_to_laserscan'
    )
    
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch', 'navigation_launch.py',
            ])
        ]),
        launch_arguments={
            # 'namespace': 'scout_mini',
            'use_sim_time' : 'False',
            'params_file' : str(os.path.join(scout_mini_control_pkg, 'config', 'nav2.yaml')),
            # 'container_name' : 'scout_mini',

        }.items(),
    )

    nav2_map_server = Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': False},
                        {'yaml_filename': map_file}],
            condition=LaunchConfigurationEquals('mode','localization')
    )

    
    lifecycle_nav2_maps = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_mapper',
            output='screen',
            parameters=[{'use_sim_time': False},
                        {'autostart': True},
                        {'node_names': ['map_server']}],
            condition=LaunchConfigurationEquals('mode','localization')
                        
    )


    ld = LaunchDescription()
    # Launch Arguments
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(declare_mode_argument)
    ld.add_action(declare_map_file)
    

    # Navigation Nodes
    ld.add_action(start_async_slam_toolbox_node)
    ld.add_action(nav2_bringup_launch)
    ld.add_action(depthimage_to_laserscan_launch)
    ld.add_action(nav2_map_server)
    ld.add_action(lifecycle_nav2_maps)


    return ld
