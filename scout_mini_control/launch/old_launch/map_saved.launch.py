import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    map_file = os.path.join(get_package_share_directory(
        'scout_mini_control'), 'maps', 'test1.yaml') #LAUNCH ARGUMENT SET
    
    ld = LaunchDescription()


    nav2_map_server = Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': False},
                        {'yaml_filename': map_file}]
    )

    
    lifecycle_nav2_maps = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_mapper',
            output='screen',
            parameters=[{'use_sim_time': False},
                        {'autostart': True},
                        {'node_names': ['map_server']}]
                        
    )

    ld.add_action(nav2_map_server)
    ld.add_action(lifecycle_nav2_maps)

    return ld
