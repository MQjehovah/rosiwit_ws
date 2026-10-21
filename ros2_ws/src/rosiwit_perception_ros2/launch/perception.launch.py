"""Unified perception bring-up.

一键启动 AVM、地面纹理、二维码识别和健康管理器。各模块仍是独立进程，
可以单独启停；manager 只订阅心跳，不参与数据面。

Usage:
    ros2 launch rosiwit_perception_ros2 perception.launch.py
    ros2 launch rosiwit_perception_ros2 perception.launch.py \\
        enable_ground_texture:=false enable_qr:=false

    # 无相机时用合成图验证 AVM
    ros2 launch rosiwit_perception_ros2 perception.launch.py use_test_pattern:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('rosiwit_perception_ros2')
    default_params = os.path.join(pkg_share, 'config', 'perception.yaml')
    default_qr_params = os.path.join(pkg_share, 'config', 'qr_detector.yaml')

    args = [
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='Unified parameter file'),
        DeclareLaunchArgument(
            'qr_params_file', default_value=default_qr_params,
            description='Standalone QR parameter file (overrides params_file)'),
        DeclareLaunchArgument(
            'enable_avm', default_value='true',
            description='Start AVM node'),
        DeclareLaunchArgument(
            'enable_ground_texture', default_value='true',
            description='Start ground texture mapper'),
        DeclareLaunchArgument(
            'enable_qr', default_value='true',
            description='Start QR detector'),
        DeclareLaunchArgument(
            'enable_obstacles', default_value='true',
            description='Start camera obstacle detector'),
        DeclareLaunchArgument(
            'enable_manager', default_value='true',
            description='Start perception health manager'),
        DeclareLaunchArgument(
            'use_test_pattern', default_value='false',
            description='AVM synthetic test pattern (no cameras required)'),
    ]

    nodes = [
        Node(
            package='rosiwit_perception_ros2',
            executable='avm_node_wrapper',
            name='avm_node',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {'use_test_pattern': ParameterValue(
                    LaunchConfiguration('use_test_pattern'), value_type=bool)},
            ],
            condition=IfCondition(LaunchConfiguration('enable_avm')),
        ),
        Node(
            package='rosiwit_perception_ros2',
            executable='ground_texture_mapper_wrapper',
            name='ground_texture_mapper',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
            condition=IfCondition(LaunchConfiguration('enable_ground_texture')),
        ),
        Node(
            package='rosiwit_perception_ros2',
            executable='qr_detector_node_wrapper',
            name='qr_detector',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                LaunchConfiguration('qr_params_file'),
            ],
            condition=IfCondition(LaunchConfiguration('enable_qr')),
        ),
        Node(
            package='rosiwit_perception_ros2',
            executable='obstacle_detector_node_wrapper',
            name='obstacle_detector',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
            condition=IfCondition(LaunchConfiguration('enable_obstacles')),
        ),
        Node(
            package='rosiwit_perception_ros2',
            executable='perception_manager_wrapper',
            name='perception_manager',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
            condition=IfCondition(LaunchConfiguration('enable_manager')),
        ),
    ]

    return LaunchDescription(args + nodes)
