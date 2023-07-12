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


    
    ld = LaunchDescription()

    scout_mini_control_dir = get_package_share_directory('scout_mini_control')

    waypoints_file = LaunchConfiguration('waypoints_file')
    rth = LaunchConfiguration('rth')
    home_point = LaunchConfiguration('home_point')

    declare_waypoints_file_cmd = DeclareLaunchArgument(
        'waypoints_file',
        default_value=os.path.join(scout_mini_control_dir, 'maps', 'waypoints.txt'),
        description='File to waypoints file',
    )

    declare_rth_cmd = DeclareLaunchArgument(
        'rth',
        default_value='True',
        description='Whether or not to RTH after completing list of waypoints',
    )

    declare_home_point_cmd = DeclareLaunchArgument(
        'home_point',
        default_value='[0.0,0.0,0.0,0.0,0.0,0.0,1.0]',
        description= 'The designated HOME point within your map'
    )


    waypoint_follower = Node(
        package='scout_mini_control',
        executable='waypoint_follower',
        parameters=[
            {'waypoints_file': waypoints_file},
            {'rth': rth},
            {'home_point': home_point}
        ]
    )

    ld.add_action(declare_waypoints_file_cmd)
    ld.add_action(declare_rth_cmd)
    ld.add_action(declare_home_point_cmd)

    ld.add_action(waypoint_follower)

    return ld