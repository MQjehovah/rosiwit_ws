"""Start only the frontier exploration node.

Use this when the simulator, SLAM and navigation are already running.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_dir = get_package_share_directory('rosiwit_exploration_ros2')

    declared_arguments = [
        DeclareLaunchArgument(
            'config_file',
            default_value=os.path.join(
                package_dir, 'config', 'exploration_params.yaml'),
            description='Exploration parameter file'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use the Gazebo /clock'),
        DeclareLaunchArgument(
            'goal_interface', default_value='pose',
            description='pose | nav2 | auto'),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Start exploring without waiting for ~/set_enabled'),
        DeclareLaunchArgument(
            'start_delay', default_value='5.0',
            description='Seconds to wait before the first frontier goal'),
        DeclareLaunchArgument(
            'save_map_on_finish', default_value='false',
            description='Save the final /map with map_saver_cli'),
        DeclareLaunchArgument(
            'save_map_path', default_value='/tmp/rosiwit_exploration_map',
            description='Map basename passed to map_saver_cli -f'),
    ]

    explorer = Node(
        package='rosiwit_exploration_ros2',
        executable='frontier_explorer',
        name='frontier_explorer',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'), value_type=bool),
                'goal_interface': LaunchConfiguration('goal_interface'),
                'autostart': ParameterValue(
                    LaunchConfiguration('autostart'), value_type=bool),
                'start_delay': ParameterValue(
                    LaunchConfiguration('start_delay'), value_type=float),
                'save_map_on_finish': ParameterValue(
                    LaunchConfiguration('save_map_on_finish'), value_type=bool),
                'save_map_path': LaunchConfiguration('save_map_path'),
            },
        ],
    )

    return LaunchDescription(declared_arguments + [explorer])
