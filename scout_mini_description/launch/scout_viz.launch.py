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

    pkg_scout_mini_description = get_package_share_directory(
        'scout_mini_description')
    scout_install_dir = get_package_prefix('scout_mini_description')
    zed_install_dir = get_package_prefix('zed_interfaces')

    visualize_arg = DeclareLaunchArgument('visualize', default_value='True', description='Launching the visualization components of the simulation')
    visualize = LaunchConfiguration('visualize')


    # Gazebo environment sourcing
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] = os.environ['GAZEBO_MODEL_PATH'] + \
            ':' + scout_install_dir + '/share' + \
            ':' + zed_install_dir + '/share'
    else:
        os.environ['GAZEBO_MODEL_PATH'] = scout_install_dir + "/share" + \
            ':' + zed_install_dir + '/share'
    if 'GAZEBO_PLUGIN_PATH' in os.environ:
        os.environ['GAZEBO_PLUGIN_PATH'] = os.environ['GAZEBO_PLUGIN_PATH'] + \
            ':' + scout_install_dir + '/lib' + \
            ':' + zed_install_dir + '/lib'
    else:
        os.environ['GAZEBO_PLUGIN_PATH'] = scout_install_dir + '/lib' + \
            ':' + zed_install_dir + '/lib'

    xacro_file = os.path.join(
        pkg_scout_mini_description, 'models/scout_mini/xacro', 'scout_mini.xacro')
    assert os.path.exists(
        xacro_file), "The scout_mini.xacro doesn't exist in " + str(xacro_file)
    
    xacro_file_viz = os.path.join(
        pkg_scout_mini_description, 'models/scout_mini/xacro', 'scout_mini_viz.xacro')
    assert os.path.exists(
        xacro_file), "The scout_mini_viz.xacro doesn't exist in " + str(xacro_file)

    robot_description_config = xacro.process_file(xacro_file)
    robot_description = robot_description_config.toxml()
    robot_description_param = {'robot_description': robot_description}

    robot_description_viz_config = xacro.process_file(xacro_file_viz)
    robot_description_viz = robot_description_viz_config.toxml()
    robot_description_viz_param = {'robot_description': robot_description_viz}

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='scout_mini',
        parameters=[robot_description_param],
        condition=UnlessCondition(visualize)
    )
    robot_state_publisher_viz_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='scout_mini',
        parameters=[robot_description_viz_param],
        condition=IfCondition(visualize)
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
            # 'world': TextSubstitution(text=str(gazebo_world))
        }.items(),
        condition=IfCondition(visualize)
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory(
            'scout_mini_control'), 'config', 'nav2_visualization.rviz')],
        condition=IfCondition(visualize)
    )

    spawn_tracer_node = Node(
        package='scout_mini_description',
        executable='spawn_scout_mini',
        namespace='/scout_mini',
        arguments=[robot_description],
        output='screen',
        condition=IfCondition(visualize)
    )

    # Adding arguments
    ld.add_action(visualize_arg)

    #Launching robot tf broadcaster
    ld.add_action(robot_state_publisher_node)

    #Launching visualization
    ld.add_action(gzserver_launch)
    ld.add_action(spawn_tracer_node)
    ld.add_action(rviz2_node)
    return ld
