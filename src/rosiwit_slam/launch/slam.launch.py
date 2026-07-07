# rosiwit_slam Launch File
# slam_algorithm 参数选择算法 (SlamFactory 按名创建), 当前可用: fast_lio2

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    slam_algorithm_arg = DeclareLaunchArgument(
        'slam_algorithm', default_value='fast_lio2',
        description='SLAM algorithm name, resolved by SlamFactory at runtime')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false')

    lidar_topic_arg = DeclareLaunchArgument(
        'lidar_topic', default_value='/velodyne_points')

    imu_topic_arg = DeclareLaunchArgument(
        'imu_topic', default_value='/imu')

    from ament_index_python.packages import get_package_share_directory
    config_dir = os.path.join(
        get_package_share_directory('rosiwit_slam'), 'config')

    slam_node = Node(
        package='rosiwit_slam',
        executable='rosiwit_slam',
        name='rosiwit_slam',
        output='screen',
        parameters=[{
            'slam_algorithm': LaunchConfiguration('slam_algorithm'),
            'config_path': os.path.join(config_dir, 'default.yaml'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'lidar_topic': LaunchConfiguration('lidar_topic'),
            'imu_topic': LaunchConfiguration('imu_topic'),
        }]
    )

    return LaunchDescription([
        slam_algorithm_arg,
        use_sim_time_arg,
        lidar_topic_arg,
        imu_topic_arg,
        slam_node
    ])
