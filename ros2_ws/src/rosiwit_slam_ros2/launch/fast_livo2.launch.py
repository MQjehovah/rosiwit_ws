# FAST-LIVO2 专用管线：播包出图（odom/path/点云/彩色地图，退出时存 PCD）
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('rosiwit_slam_ros2')

    config_arg = DeclareLaunchArgument(
        'config_path',
        default_value=os.path.join(pkg, 'config', 'fast_livo2_ntu.yaml'),
        description='FAST-LIVO2 config (camera/extrinsics/topics)')
    sim_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    rviz_arg = DeclareLaunchArgument('use_rviz', default_value='false')
    pcd_arg = DeclareLaunchArgument(
        'output_pcd', default_value=os.path.join(os.getcwd(), 'livo_map.pcd'))

    livo_node = Node(
        package='rosiwit_slam_ros2',
        executable='livo_node',
        name='fast_livo2',
        output='screen',
        parameters=[{
            'config_path': LaunchConfiguration('config_path'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'output_pcd': LaunchConfiguration('output_pcd'),
        }],
    )

    rviz = Node(
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg, 'rviz', 'slam.rviz')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen',
    )

    return LaunchDescription([config_arg, sim_arg, rviz_arg, pcd_arg, livo_node, rviz])
