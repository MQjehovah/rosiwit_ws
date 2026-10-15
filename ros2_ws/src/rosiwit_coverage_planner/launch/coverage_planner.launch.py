#!/usr/bin/env python3
"""
ROS2 Coverage Planner Launch File

Launches the coverage planner node with configurable parameters.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    coverage_mode_arg = DeclareLaunchArgument(
        'coverage_mode',
        default_value='zigzag',
        description='Coverage planning mode: zigzag or spiral'
    )
    
    robot_radius_arg = DeclareLaunchArgument(
        'robot_radius',
        default_value='0.3',
        description='Robot radius in meters for obstacle inflation'
    )
    
    coverage_resolution_arg = DeclareLaunchArgument(
        'coverage_resolution',
        default_value='0.1',
        description='Coverage path resolution in meters'
    )
    
    enable_optimization_arg = DeclareLaunchArgument(
        'enable_optimization',
        default_value='true',
        description='Enable path optimization (simplification)'
    )
    
    direction_optimization_arg = DeclareLaunchArgument(
        'direction_optimization',
        default_value='2',
        description='Scan direction: 0=horizontal, 1=vertical, 2=auto'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='map',
        description='Reference frame for the coverage path'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('ros2_coverage_planner'),
            'config',
            'coverage_params.yaml'
        ]),
        description='Path to the parameter file'
    )
    
    # Coverage planner node
    coverage_planner_node = Node(
        package='ros2_coverage_planner',
        executable='coverage_planner_node',
        name='coverage_planner_node',
        output='screen',
        parameters=[
            # Use params file if available
            LaunchConfiguration('params_file'),
            # Override with launch arguments
            {'coverage_mode': LaunchConfiguration('coverage_mode')},
            {'robot_radius': LaunchConfiguration('robot_radius')},
            {'coverage_resolution': LaunchConfiguration('coverage_resolution')},
            {'enable_optimization': LaunchConfiguration('enable_optimization')},
            {'direction_optimization': LaunchConfiguration('direction_optimization')},
            {'frame_id': LaunchConfiguration('frame_id')},
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        remappings=[
            # Optional: remap topics if needed
            # ('/map', '/your_map_topic'),
            # ('/coverage_path', '/your_path_topic'),
        ],
    )
    
    return LaunchDescription([
        coverage_mode_arg,
        robot_radius_arg,
        coverage_resolution_arg,
        enable_optimization_arg,
        direction_optimization_arg,
        frame_id_arg,
        use_sim_time_arg,
        params_file_arg,
        coverage_planner_node,
    ])