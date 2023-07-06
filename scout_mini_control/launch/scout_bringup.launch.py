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
from launch.conditions import IfCondition, UnlessCondition, LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch.substitutions import LaunchConfiguration


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes, SetParameter
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from nav2_common.launch import RewrittenYaml
from launch_ros.actions import PushRosNamespace, SetRemap


def generate_launch_description():

    # Get required directories
    scout_mini_control_dir = get_package_share_directory(
        'scout_mini_control')
    scout_mini_description_dir = get_package_share_directory(
        'scout_mini_description')

    # Create the launch configuration variables
    ekf_params = LaunchConfiguration('ekf_params')
    namespace = LaunchConfiguration('namespace')
    localization = LaunchConfiguration('localization')
    use_sim_time = LaunchConfiguration('use_sim_time')
    database_path = LaunchConfiguration('database_path')
    # Declaring Launch Arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='scout_mini',
        description='Namespace to use with nodes'
    )
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

    # robot_state_publisher_node = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     name='robot_state_publisher',
    #     namespace='/scout_mini',
    #     parameters=[robot_description_param],
    # )

    # Navigation Launch

    slam_include = GroupAction(
        condition=LaunchConfigurationNotEquals('localization','true'),
        actions=[
            PushRosNamespace(
                namespace=namespace),
            SetParameter('use_sim_time', use_sim_time),

            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                namespace=namespace,
                parameters=[robot_description_param]),

            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_filter_node',
                output='screen',
                namespace=namespace,
                parameters=[ekf_params]),
            
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('rtabmap_launch'),
                        'launch', 'rtabmap.launch.py',
                    ])]),
                launch_arguments={
                    'namespace': namespace,
                    'use_sim_time' : use_sim_time,
                    'rtabmap_args' : '--delete_db_on_start',
                    'database_path' : database_path,
                    'rgb_topic' : '/scout_mini/zed_node/rgb/image_rect_color',
                    'depth_topic' : '/scout_mini/zed_node/depth/depth_registered',
                    'camera_info_topic' : '/scout_mini/zed_node/rgb/camera_info',
                    'frame_id' : 'base_footprint',
                    'approx_sync' : 'false',
                    'wait_imu_to_init' : 'false',
                    'wait_for_transform' : '0.3',
                    'imu_topic' : '/scout_mini/zed_node/imu/data',
                    'odom_frame_id' : 'odom',
                    'qos' : '1',
                    'rtabmapviz' : 'false',
                    'rviz' : 'false',
                    'localization' : localization}.items()),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('microstrain_inertial_driver'),
                        'launch', 'microstrain_launch.py'])]),
                    launch_arguments={
                        'namespace': namespace}.items())
            ]
        )
    
    localization_include = GroupAction(
        condition=LaunchConfigurationEquals('localization', 'true'),
        actions=[
            PushRosNamespace(
                namespace=namespace),
            SetParameter('use_sim_time', use_sim_time),

            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                namespace=namespace,
                parameters=[robot_description_param]),

            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_filter_node',
                output='screen',
                namespace=namespace,
                parameters=[ekf_params]),
            
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('rtabmap_launch'),
                        'launch', 'rtabmap.launch.py',
                    ])]),
                launch_arguments={
                    'namespace': namespace,
                    'use_sim_time' : use_sim_time,
                    'rtabmap_args' : '',
                    'database_path' : database_path,
                    'rgb_topic' : '/scout_mini/zed_node/rgb/image_rect_color',
                    'depth_topic' : '/scout_mini/zed_node/depth/depth_registered',
                    'camera_info_topic' : '/scout_mini/zed_node/rgb/camera_info',
                    'frame_id' : 'base_footprint',
                    'approx_sync' : 'false',
                    'wait_imu_to_init' : 'false',
                    'wait_for_transform' : '0.3',
                    'imu_topic' : '/scout_mini/zed_node/imu/data',
                    'odom_frame_id' : 'odom',
                    'qos' : '1',
                    'rtabmapviz' : 'false',
                    'rviz' : 'false',
                    'localization' : localization}.items()),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('microstrain_inertial_driver'),
                        'launch', 'microstrain_launch.py'])]),
                    launch_arguments={
                        'namespace': namespace}.items())
            ]
        )
    
    # robot_localization_node = Node(
    #     package='robot_localization',
    #     executable='ekf_node',
    #     name='ekf_filter_node',
    #     output='screen',
    #     namespace='/scout_mini',
    #     parameters=[ekf_params]
    # )

    # rtabmap_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         PathJoinSubstitution([
    #             FindPackageShare('rtabmap_launch'),
    #             'launch', 'rtabmap.launch.py',
    #         ])
    #         ]),
    #         launch_arguments={
    #             'namespace': 'scout_mini/rtabmap',
    #             'use_sim_time' : use_sim_time,
    #             'rtabmap_args' : rtabmap_args,
    #             'database_path' : database_path,
    #             'rgb_topic' : '/scout_mini/zed_node/rgb/image_rect_color',
    #             'depth_topic' : '/scout_mini/zed_node/depth/depth_registered',
    #             'camera_info_topic' : '/scout_mini/zed_node/rgb/camera_info',
    #             'frame_id' : 'base_footprint',
    #             'approx_sync' : 'false',
    #             'wait_imu_to_init' : 'false',
    #             'wait_for_transform' : '0.3',
    #             'imu_topic' : '/scout_mini/zed_node/imu/data',
    #             'odom_frame_id' : 'odom',
    #             'qos' : '1',
    #             'rtabmapviz' : 'false',
    #             'rviz' : 'false',
    #             'localization' : localization,
    #         }.items(),
    # )

    # Sensor Launch

    # microstrain_imu_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         PathJoinSubstitution([
    #             FindPackageShare('microstrain_inertial_driver'),
    #             'launch', 'microstrain_launch.py',
    #         ])
    #     ]),
    #     launch_arguments={
    #         'namespace': 'scout_mini',
    #     }.items(),
    # )


    # Adding arguments
    ld.add_action(declare_ekf_params)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_localization_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_database_path_cmd)

    # Navigation Nodes
    ld.add_action(slam_include)
    ld.add_action(localization_include)


    #ld.add_action(robot_localization_node)
    #ld.add_action(rtabmap_launch)

    # Robot TF Launch
    #ld.add_action(robot_state_publisher_node)

    # Sensor launch
    #ld.add_action(microstrain_imu_launch)
    # ld.add_action(depthimage_to_laserscan_launch)

    return ld
