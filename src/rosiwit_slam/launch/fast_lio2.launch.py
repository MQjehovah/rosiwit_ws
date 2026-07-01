# FAST-LIO2 SLAM Launch File

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 声明launch参数
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='default.yaml',
        description='Configuration file name'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    lidar_topic_arg = DeclareLaunchArgument(
        'lidar_topic',
        default_value='/velodyne_points',
        description='LiDAR point cloud topic'
    )

    imu_topic_arg = DeclareLaunchArgument(
        'imu_topic',
        default_value='/imu',
        description='IMU data topic'
    )

    # 配置文件路径
    config_file_path = PathJoinSubstitution([
        FindPackageShare('rosiwit_slam'),
        'config',
        LaunchConfiguration('config_file')
    ])

    # 创建节点
    slam_node = Node(
        package='rosiwit_slam',
        executable='rosiwit_slam',
        name='rosiwit_slam',
        output='screen',
        parameters=[
            {'config_file': config_file_path},
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'lidar_topic': LaunchConfiguration('lidar_topic')},
            {'imu_topic': LaunchConfiguration('imu_topic')}
        ]
    )

    return LaunchDescription([
        config_file_arg,
        use_sim_time_arg,
        lidar_topic_arg,
        imu_topic_arg,
        slam_node
    ])