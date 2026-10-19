# rosiwit_slam Launch File
# slam_algorithm 参数选择算法 (SlamFactory 按名创建)
#   当前可用: fast_lio2_pipeline

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    slam_algorithm_arg = DeclareLaunchArgument(
        'slam_algorithm', default_value='fast_lio2_pipeline',
        description='SLAM algorithm name, resolved by SlamFactory at runtime')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false')

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Launch rviz2 with the package rviz config')

    use_mt_arg = DeclareLaunchArgument(
        'use_multi_threaded', default_value='true',
        description='Use MultiThreadedExecutor (decouple sensor RX from heavy processing to avoid dropped scans)')

    config_arg = DeclareLaunchArgument(
        'config', default_value='default.yaml',
        description='Config file name under <pkg>/config (e.g. default.yaml, livox.yaml, sim_vlp16.yaml, vanjee722.yaml)')

    from ament_index_python.packages import get_package_share_directory
    from launch_ros.substitutions import FindPackageShare
    from launch.substitutions import PathJoinSubstitution

    pkg_share = get_package_share_directory('rosiwit_slam_ros2')

    config_default = PathJoinSubstitution(
        [FindPackageShare('rosiwit_slam_ros2'), 'config', LaunchConfiguration('config')])
    rviz_config = os.path.join(pkg_share, 'rviz', 'slam.rviz')

    slam_node = Node(
        package='rosiwit_slam_ros2',
        executable='rosiwit_slam_ros2',
        name='rosiwit_slam',
        output='screen',
        parameters=[{
            'slam_algorithm': LaunchConfiguration('slam_algorithm'),
            'config_path': config_default,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'use_multi_threaded': LaunchConfiguration('use_multi_threaded'),
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
        use_mt_arg,
        config_arg,
        slam_node,
        rviz_node,
    ])
