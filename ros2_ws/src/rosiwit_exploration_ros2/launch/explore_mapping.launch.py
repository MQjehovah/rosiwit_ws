"""Full simulation + SLAM + navigation + frontier exploration.

Pipeline:
  Gazebo (robot + world)
    -> rosiwit_slam_ros2 (live /map + odom -> base_link)
    -> rosiwit_navigation_ros2 (A*/DWA, consumes /map and /goal_pose)
    -> rosiwit_exploration_ros2 (frontier goals on /goal_pose)

For a Nav2 stack instead, set goal_interface:=nav2 and goal_frame:=map.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    exploration_dir = get_package_share_directory('rosiwit_exploration_ros2')
    simulator_dir = get_package_share_directory('rosiwit_simulator')
    slam_dir = get_package_share_directory('rosiwit_slam_ros2')
    navigation_dir = get_package_share_directory('rosiwit_navigation_ros2')

    default_world = os.path.join(simulator_dir, 'world', 'house.world')

    declared_arguments = [
        DeclareLaunchArgument(
            'world_name', default_value=default_world,
            description='Gazebo world used for exploration'),
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='Show the Gazebo client'),
        DeclareLaunchArgument(
            'spawn_x', default_value='0.0',
            description='Robot spawn x (house.world uses 0)'),
        DeclareLaunchArgument(
            'spawn_y', default_value='0.0',
            description='Robot spawn y (house.world uses 0)'),
        DeclareLaunchArgument(
            'spawn_z', default_value='0.1',
            description='Robot spawn z'),
        DeclareLaunchArgument(
            'spawn_yaw', default_value='0.0',
            description='Robot spawn yaw'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Start RViz with the SLAM configuration'),
        DeclareLaunchArgument(
            'slam_config', default_value='simulator.yaml',
            description='Config file under rosiwit_slam_ros2/config'),
        DeclareLaunchArgument(
            'exploration_config',
            default_value=os.path.join(
                exploration_dir, 'config', 'exploration_params.yaml'),
            description='Frontier exploration parameter file'),
        DeclareLaunchArgument(
            'goal_interface', default_value='pose',
            description='pose | nav2 | auto'),
        DeclareLaunchArgument(
            'goal_frame', default_value='odom',
            description='Frame used for goals (use map with Nav2)'),
        DeclareLaunchArgument(
            'start_delay', default_value='15.0',
            description='Delay before the explorer starts sending goals'),
        DeclareLaunchArgument(
            'save_map_on_finish', default_value='false',
            description='Save the map when exploration finishes'),
        DeclareLaunchArgument(
            'save_map_path', default_value='/tmp/rosiwit_exploration_map',
            description='Map basename passed to map_saver_cli -f'),
    ]

    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(simulator_dir, 'launch', 'simulator_gazebo.launch.py')),
        launch_arguments={
            'world_name': LaunchConfiguration('world_name'),
            'gui': LaunchConfiguration('gui'),
            'use_sim_time': 'true',
            'spawn_x': LaunchConfiguration('spawn_x'),
            'spawn_y': LaunchConfiguration('spawn_y'),
            'spawn_z': LaunchConfiguration('spawn_z'),
            'spawn_yaw': LaunchConfiguration('spawn_yaw'),
        }.items(),
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_dir, 'launch', 'slam.launch.py')),
        launch_arguments={
            'config': LaunchConfiguration('slam_config'),
            'use_sim_time': 'true',
            'use_rviz': LaunchConfiguration('use_rviz'),
        }.items(),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_dir, 'launch', 'navigation.launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'use_rviz': 'false',
            'load_map': 'false',
        }.items(),
    )

    explorer = Node(
        package='rosiwit_exploration_ros2',
        executable='frontier_explorer',
        name='frontier_explorer',
        output='screen',
        parameters=[
            LaunchConfiguration('exploration_config'),
            {
                'use_sim_time': True,
                'goal_interface': LaunchConfiguration('goal_interface'),
                'goal_frame': LaunchConfiguration('goal_frame'),
                'autostart': True,
                'start_delay': ParameterValue(
                    LaunchConfiguration('start_delay'), value_type=float),
                'save_map_on_finish': ParameterValue(
                    LaunchConfiguration('save_map_on_finish'), value_type=bool),
                'save_map_path': LaunchConfiguration('save_map_path'),
            },
        ],
    )

    return LaunchDescription(
        declared_arguments
        + [simulator, slam, navigation]
        + [
            TimerAction(
                period=LaunchConfiguration('start_delay'),
                actions=[explorer],
            )
        ]
    )
