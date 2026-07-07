# rosiwit_slam Launch File
# slam_algorithm 参数选择算法 (SlamFactory 按名创建), 当前可用: fast_lio2

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    slam_algorithm_arg = DeclareLaunchArgument(
        'slam_algorithm', default_value='fast_lio2',
        description='SLAM algorithm name, resolved by SlamFactory at runtime')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false')

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Launch rviz2 with the package rviz config (subscribes cloud_map to trigger publishing)')

    lidar_topic_arg = DeclareLaunchArgument(
        'lidar_topic', default_value='/velodyne_points')

    imu_topic_arg = DeclareLaunchArgument(
        'imu_topic', default_value='/imu')

    from ament_index_python.packages import get_package_share_directory
    pkg_share = get_package_share_directory('rosiwit_slam')
    config_default = os.path.join(pkg_share, 'config', 'default.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'slam.rviz')

    slam_node = Node(
        package='rosiwit_slam',
        executable='rosiwit_slam',
        name='rosiwit_slam',
        output='screen',
        parameters=[{
            'slam_algorithm': LaunchConfiguration('slam_algorithm'),
            'config_path': config_default,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'lidar_topic': LaunchConfiguration('lidar_topic'),
            'imu_topic': LaunchConfiguration('imu_topic'),
        }]
    )

    # rviz2 订阅 /cloud_map, 触发节点的 lazy-publish (否则全局地图不会发布)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription([
        slam_algorithm_arg,
        use_sim_time_arg,
        use_rviz_arg,
        lidar_topic_arg,
        imu_topic_arg,
        slam_node,
        rviz_node,
    ])
