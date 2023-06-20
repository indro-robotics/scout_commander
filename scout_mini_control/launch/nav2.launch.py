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

    scout_mini_control_pkg = get_package_share_directory('scout_mini_control')

    ekf_config = os.path.join(scout_mini_control_pkg, 'config/ekf.yaml')
    sim_time_argument = DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                              description='Flag to enable use_sim_time')


    
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        namespace='/scout_mini',
        parameters=[ekf_config]
    )


    # Navigation Nodes
    ld.add_action(robot_localization_node)
    ld.add_action(sim_time_argument)

    return ld
