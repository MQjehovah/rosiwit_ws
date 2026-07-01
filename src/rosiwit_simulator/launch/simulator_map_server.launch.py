import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Map file argument
    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value='/opt/xzrobot/maps/gazebo/map.yaml'
    )

    # Map server 节点
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        arguments=[LaunchConfiguration('map_file')],
    )

    return LaunchDescription([
        map_file_arg,
        map_server,
    ])
