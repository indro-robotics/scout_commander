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

    ld = LaunchDescription()

    scout_mini_control_pkg = get_package_share_directory(
        'scout_mini_control')
    scout_mini_description_pkg = get_package_share_directory(
        'scout_mini_description')

    ekf_config = os.path.join(scout_mini_control_pkg, 'config/ekf.yaml')

    # Robot Description File
    xacro_file = os.path.join(
        scout_mini_description_pkg, 'models/scout_mini/xacro', 'scout_mini_tf.xacro')
    assert os.path.exists(
        xacro_file), "The scout_mini_tf.xacro doesn't exist in " + str(xacro_file)

    robot_description_config = xacro.process_file(xacro_file)
    robot_description = robot_description_config.toxml()
    robot_description_param = {'robot_description': robot_description}

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='/scout_mini',
        parameters=[robot_description_param],
    )
    
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        namespace='/scout_mini',
        parameters=[ekf_config]
    )

    IMU_launch = IncludeLaunchDescription(
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


    # Navigation Nodes
    ld.add_action(robot_localization_node)

    # Robot TF Launch
    ld.add_action(robot_state_publisher_node)

    # Sensor launch
    ld.add_action(IMU_launch)

    return ld
