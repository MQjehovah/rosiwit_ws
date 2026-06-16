"""
app_only.launch.py - 仅启动 App 节点
假设其他模块（simulator/slam/navigation）已经独立运行
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    app_params = PathJoinSubstitution([
        FindPackageShare('rosiwit_app'), 'config', 'app_params.yaml'
    ])

    app_node = Node(
        package='rosiwit_app',
        executable='app_node',
        name='rosiwit_app',
        output='screen',
        parameters=[
            app_params,
            {
                'map_path': LaunchConfiguration('map_path'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('map_path', default_value='/tmp/rosiwit_sim_map'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        app_node,
    ])
