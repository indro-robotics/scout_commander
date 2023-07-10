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
    namespace='scout_mini'
    # namespace = LaunchConfiguration('namespace')
    localization = LaunchConfiguration('localization')
    use_sim_time = LaunchConfiguration('use_sim_time')
    database_path = LaunchConfiguration('database_path')
    rtabmap_args = LaunchConfiguration('rtabmap_args')
    # Declaring Launch Arguments
    # declare_namespace_cmd = DeclareLaunchArgument(
    #     'namespace',
    #     default_value='',
    #     description='Namespace to use with nodes'
    # )
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
        default_value='/home/indro/colcon_ws/src/scout_commander/scout_mini_control/maps/rtabmap.db',
        description= 'Where the map is saved and loaded'
    )

    declare_rtabmap_args_cmd = DeclareLaunchArgument(
        'rtabmap_args',
        default_value='',
        description= 'RTABMap specific args to pass through (ex. --delete_db_on_start)'
    )
    # Robot Description File
    xacro_file = os.path.join(
        scout_mini_description_dir, 'models/scout_mini/xacro', 'scout_mini_tf.xacro')
    assert os.path.exists(
        xacro_file), "The scout_mini_tf.xacro doesn't exist in " + str(xacro_file)

    robot_description_config = xacro.process_file(xacro_file)
    robot_description = robot_description_config.toxml()
    robot_description_param = {'robot_description': robot_description}

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
                parameters=[robot_description_param]),

            # Node(
            #     package='robot_localization',
            #     executable='ekf_node',
            #     name='ekf_filter_node',
            #     output='screen',
            #     parameters=[ekf_params]),
            #EKF NODE HAS BEEN CAUSING BAD DRIFTING PROBLEMS

            IncludeLaunchDescription(asdfasd
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('rtabmap_launch'),
                        'launch', 'rtabmap.launch.py',
                    ])]),
                launch_arguments={
                    # 'namespace': namespace,
                    'use_sim_time' : use_sim_time,
                    'rtabmap_args' : rtabmap_args,
                    'database_path' : database_path,
                    'rgb_topic' : '/scout_mini/zed_node/rgb/image_rect_color',
                    'depth_topic' : '/scout_mini/zed_node/depth/depth_registered',
                    'camera_info_topic' : '/scout_mini/zed_node/rgb/camera_info',
                    'frame_id' : 'base_footprint',
                    'approx_sync' : 'false',
                    'wait_imu_to_init' : 'false',
                    'wait_for_transform' : '0.2',
                    'imu_topic' : '/scout_mini/zed_node/imu/data',
                    'odom_frame_id' : 'odom',
                    'qos' : '2',
                    'rtabmapviz' : 'false',
                    'rviz' : 'false',
                    'localization' : localization}.items()),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('microstrain_inertial_driver'),
                        'launch', 'microstrain_launch.py'])])),

            # IncludeLaunchDescription(
            #     PythonLaunchDescriptionSource([
            #         PathJoinSubstitution([
            #             FindPackageShare('scout_mini_control'),
            #             'launch', 'navigation_bringup.launch.py'])]))
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
                parameters=[robot_description_param]),

            # Node(
            #     package='robot_localization',
            #     executable='ekf_node',
            #     name='ekf_filter_node',
            #     output='screen',
            #     parameters=[ekf_params]),
            # EKF NODE HAS BEEN CREATING BAD DRIFING PROBLEMS
            
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('rtabmap_launch'),
                        'launch', 'rtabmap.launch.py',
                    ])]),
                launch_arguments={
                    # 'namespace': namespace,
                    'use_sim_time' : use_sim_time,
                    'rtabmap_args' : rtabmap_args,
                    'database_path' : database_path,
                    'rgb_topic' : '/scout_mini/zed_node/rgb/image_rect_color',
                    'depth_topic' : '/scout_mini/zed_node/depth/depth_registered',
                    'camera_info_topic' : '/scout_mini/zed_node/rgb/camera_info',
                    'frame_id' : 'base_footprint',
                    'approx_sync' : 'false',
                    'wait_imu_to_init' : 'false',
                    'wait_for_transform' : '0.1',
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
                        'launch', 'microstrain_launch.py'])])),

            # IncludeLaunchDescription(
            #     PythonLaunchDescriptionSource([
            #         PathJoinSubstitution([
            #             FindPackageShare('scout_mini_control'),
            #             'launch', 'navigation_bringup.launch.py'])]))
            ]
        )

    ld = LaunchDescription()
    # Adding arguments
    ld.add_action(declare_ekf_params)
    ld.add_action(declare_rtabmap_args_cmd)
    #ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_localization_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_database_path_cmd)

    # Navigation Nodes
    ld.add_action(slam_include)
    ld.add_action(localization_include)

    return ld
