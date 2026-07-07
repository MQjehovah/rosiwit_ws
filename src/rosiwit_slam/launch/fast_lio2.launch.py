# FAST-LIO2 SLAM Launch File

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
import os


def generate_launch_description():
    config_file_arg = DeclareLaunchArgument(
        'config_file', default_value='default.yaml')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false')

    lidar_topic_arg = DeclareLaunchArgument(
        'lidar_topic', default_value='/velodyne_points')

    imu_topic_arg = DeclareLaunchArgument(
        'imu_topic', default_value='/imu')

    # Resolve config path at launch time
    from ament_index_python.packages import get_package_share_directory
    config_dir = os.path.join(
        get_package_share_directory('rosiwit_slam'), 'config')

    slam_node = Node(
        package='rosiwit_slam',
        executable='rosiwit_slam',
        name='rosiwit_slam',
        output='screen',
        parameters=[{
            'config_path': os.path.join(config_dir, 'default.yaml'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'lidar_topic': LaunchConfiguration('lidar_topic'),
            'imu_topic': LaunchConfiguration('imu_topic'),
        }]
    )

    return LaunchDescription([
        config_file_arg,
        use_sim_time_arg,
        lidar_topic_arg,
        imu_topic_arg,
        slam_node
    ])