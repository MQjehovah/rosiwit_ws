# rosiwit_slam Launch File
# slam_algorithm 参数选择算法 (SlamFactory 按名创建)
#   当前可用: fast_lio2_pipeline

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
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

    # 离线播包常用: 不改 yaml, 直接覆盖话题与订阅 QoS
    image_topic_arg = DeclareLaunchArgument(
        'image_topic', default_value='',
        description='Override camera image topic (empty = use config; bags often need /left/image_raw)')
    lidar_topic_arg = DeclareLaunchArgument(
        'lidar_topic', default_value='', description='Override lidar topic (empty = use config)')
    imu_topic_arg = DeclareLaunchArgument(
        'imu_topic', default_value='', description='Override IMU topic (empty = use config)')
    qos_arg = DeclareLaunchArgument(
        'qos_reliable', default_value='-1',
        description='Override sensor QoS: -1 = use config, 1 = reliable (bag playback), 0 = best_effort')

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
            'image_topic': LaunchConfiguration('image_topic'),
            'lidar_topic': LaunchConfiguration('lidar_topic'),
            'imu_topic': LaunchConfiguration('imu_topic'),
            'qos_reliable': ParameterValue(LaunchConfiguration('qos_reliable'), value_type=int),
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
        image_topic_arg,
        lidar_topic_arg,
        imu_topic_arg,
        qos_arg,
        slam_node,
        rviz_node,
    ])
