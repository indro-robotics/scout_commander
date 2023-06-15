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

    xacro_file = os.path.join(
        pkg_scout_mini_description, 'models/scout_mini/xacro', 'scout_mini_viz.xacro')
    assert os.path.exists(
        xacro_file), "The scout_mini_viz.xacro doesn't exist in " + str(xacro_file)

    robot_description_config = xacro.process_file(xacro_file)
    robot_description = robot_description_config.toxml()
    robot_description_param = {'robot_description': robot_description}

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='/scout_mini',
        parameters=[robot_description_param],
        #condition=UnlessCondition(visualize)
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

    # Adding arguments

    #Launching robot tf broadcaster
    ld.add_action(robot_state_publisher_node)
    ld.add_action(IMU_launch)
    return ld
