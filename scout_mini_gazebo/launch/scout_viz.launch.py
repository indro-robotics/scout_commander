#!/usr/bin/env python3

import os
import sys

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro

from launch.substitutions import PathJoinSubstitution, TextSubstitution


from launch_ros.substitutions import FindPackageShare

from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution
)

from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess
)


def generate_launch_description():

    ld = LaunchDescription()

    sim_time_argument = DeclareLaunchArgument(name='use_sim_time', default_value='False',
                                              description='Flag to enable use_sim_time')

    scout_description_install_dir = get_package_prefix(
        'scout_mini_description')
    scout_gazebo_install_dir = get_package_prefix('scout_mini_gazebo')
    zed_wrapper_install_dir = get_package_prefix('zed_wrapper')

    

    scout_description_share_dir = get_package_share_directory(
        'scout_mini_description')
    scout_gazebo_share_dir = get_package_share_directory('scout_mini_gazebo')
    scout_control_share_dir = get_package_share_directory('scout_mini_control')


    # Gazebo environment sourcing
    gazebo_world = os.path.join(
        scout_gazebo_share_dir, 'worlds', 'obstacle_simulation.sdf')
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] = os.environ['GAZEBO_MODEL_PATH'] + \
            ':' + scout_description_install_dir + '/share' + \
            ':' + zed_wrapper_install_dir + '/share' + \
            ':' + scout_gazebo_install_dir + '/share'
    else:
        os.environ['GAZEBO_MODEL_PATH'] = scout_description_install_dir + "/share" + \
            ':' + zed_wrapper_install_dir + '/share' + \
            ':' + scout_gazebo_install_dir + '/share'
    if 'GAZEBO_PLUGIN_PATH' in os.environ:
        os.environ['GAZEBO_PLUGIN_PATH'] = os.environ['GAZEBO_PLUGIN_PATH'] + \
            ':' + scout_description_install_dir + '/lib' + \
            ':' + zed_wrapper_install_dir + '/lib' + \
            ':' + scout_gazebo_install_dir + '/lib'
    else:
        os.environ['GAZEBO_PLUGIN_PATH'] = scout_description_install_dir + '/lib' + \
            ':' + zed_wrapper_install_dir + '/lib' + \
            ':' + scout_gazebo_install_dir

    # Robot Description XACRO
    xacro_file = os.path.join(
        scout_gazebo_share_dir, 'models/scout_mini/xacro', 'scout_mini.xacro')
    assert os.path.exists(
        xacro_file), "The scout_mini.xacro doesn't exist in " + str(xacro_file)

    robot_description_config = xacro.process_file(xacro_file)
    robot_description = robot_description_config.toxml()
    robot_description_param = {'robot_description': robot_description}

    # Declaring Nodes

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='scout_mini',
        parameters=[robot_description_param],
    )

    gzserver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch', 'gzserver.launch.py',
            ])
        ]),
        launch_arguments={
            'verbose': 'true',
            'world': TextSubstitution(text=str(gazebo_world))
        }.items(),
    )

    gzclient_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch', 'gzclient.launch.py'
            ])
        ]),
        launch_arguments={
            'verbose': 'false',
        }.items()
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory(
            'scout_mini_description'), 'config', 'nav2_visualization.rviz')],
    )

    spawn_tracer_node = Node(
        package='scout_mini_description',
        executable='spawn_scout_mini',
        namespace='/scout_mini',
        arguments=[robot_description],
        output='screen',
    )
    scout_mini_control_pkg = get_package_share_directory('scout_mini_control')

    ekf_config = os.path.join(scout_mini_control_pkg, 'config/ekf.yaml')

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        namespace='/scout_mini',
        parameters=[ekf_config, {
            'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    pointcloud_to_laserscan_launch = Node(
        package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
        remappings=[('cloud_in', 'scout_mini/zed_depth_camera/points'),
                    ('scan', 'scout_mini/scan')],
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
                'inf_epsilon': 1.0,
                'use_sim_time' : True
        }],
        name='pointcloud_to_laserscan'
    )

    # Adding arguments
    ld.add_action(sim_time_argument)

    # Navigation Nodes
    ld.add_action(robot_localization_node)
    ld.add_action(pointcloud_to_laserscan_launch)

    # Launching robot TF broadcaster
    ld.add_action(robot_state_publisher_node)

    # Launching visualization
    ld.add_action(gzserver_launch)
    ld.add_action(gzclient_launch)
    ld.add_action(spawn_tracer_node)
    ld.add_action(rviz2_node)

    return ld
