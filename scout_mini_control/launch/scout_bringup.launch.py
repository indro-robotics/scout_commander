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

    # Get required directories
    scout_mini_control_dir = get_package_share_directory(
        'scout_mini_control')
    scout_mini_description_dir = get_package_share_directory(
        'scout_mini_description')

    # Create the launch configuration variables
    ekf_params = LaunchConfiguration('ekf_params')
    localization = LaunchConfiguration('localization')
    use_sim_time = LaunchConfiguration('use_sim_time')
    rtabmap_args = LaunchConfiguration('rtabmap_args')
    database_path = LaunchConfiguration('database_path')
    # Declaring Launch Arguments
    declare_ekf_params = DeclareLaunchArgument(
        'ekf_params',
        default_value=os.path.join(scout_mini_control_dir, 'params','ekf_params.yaml'),
        description='File path to EKF_Node parameters',
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    
    declare_localization_cmd = DeclareLaunchArgument(
        'localization',
        default_value='false',
        description='Launch in localization mode.'
    )
    declare_rtabmap_args_cmd = DeclareLaunchArgument(
        'rtabmap_args',
        default_value='',
        description='Rtabmap specific args to use'
    )
    
    declare_database_path_cmd = DeclareLaunchArgument(
        'database_path',
        default_value=os.path.join(scout_mini_control_dir, 'maps', 'rtabmap.db'),
        description= 'Where the map is saved and loaded'
    )
    # Robot Description File
    xacro_file = os.path.join(
        scout_mini_description_dir, 'models/scout_mini/xacro', 'scout_mini_tf.xacro')
    assert os.path.exists(
        xacro_file), "The scout_mini_tf.xacro doesn't exist in " + str(xacro_file)

    robot_description_config = xacro.process_file(xacro_file)
    robot_description = robot_description_config.toxml()
    robot_description_param = {'robot_description': robot_description}


    ld = LaunchDescription()
    # Robot Description Launch

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='/scout_mini',
        parameters=[robot_description_param],
    )

    # Navigation Launch
    
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        namespace='/scout_mini',
        parameters=[ekf_params]
    )

    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('rtabmap_launch'),
                'launch', 'rtabmap.launch.py',
            ])
        ]),
        launch_arguments={
            'namespace': 'scout_mini',
            'use_sim_time' : use_sim_time,
            'rtabmap_args' : rtabmap_args,
            'database_path' : database_path,
            'rgb_topic' : '/scout_mini/zed_node/rgb/image_rect_color',
            'depth_topic' : '/scout_mini/zed_node/depth/depth_registered',
            'camera_info_topic' : '/scout_mini/zed_node/rgb/camera_info',
            'frame_id' : 'base_footprint',
            'approx_sync' : 'false',
            'wait_imu_to_init' : 'false',
            'imu_topic' : '/scout_mini/imu/data',
            'odom_frame_id' : 'odom',
            'qos' : '1',
            'rtabmapviz' : 'false',
            'rviz' : 'false',
            'localization' : localization,
        }.items(),
    )

    # Sensor Launch

    microstrain_imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('microstrain_inertial_driver'),
                'launch', 'microstrain_launch.py',
            ])
        ]),
        launch_arguments={
            'namespace': 'scout_mini',
        }.items(),
    )
    # depthimage_to_laserscan_launch = Node(
    #     package='depthimage_to_laserscan', executable='depthimage_to_laserscan_node',
    #     remappings=[('depth', '/scout_mini/zed_node/depth/depth_registered'),
    #                 ('depth_camera_info', '/scout_mini/zed_node/depth/camera_info'),
    #                 ('scan', 'scout_mini/scan')],
    #     parameters=[{
    #             'range_min' : 0.2,
    #             'scan_height' : 5,
    #             'range_max' : 10.0,
    #             'scan_time' : 0.0333333,
    #             'output_frame' : 'scout_mini_right_camera_frame',
    #     }],
    #     name='depthimage_to_laserscan'
    # )

    # Adding arguments
    ld.add_action(declare_ekf_params)
    ld.add_action(declare_localization_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_rtabmap_args_cmd)
    ld.add_action(declare_database_path_cmd)

    # Navigation Nodes
    ld.add_action(robot_localization_node)
    ld.add_action(rtabmap_launch)

    # Robot TF Launch
    ld.add_action(robot_state_publisher_node)

    # Sensor launch
    ld.add_action(microstrain_imu_launch)
    # ld.add_action(depthimage_to_laserscan_launch)

    return ld
