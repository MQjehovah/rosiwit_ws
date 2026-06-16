# FAST-LIO2 SLAM with Livox Avia Launch File

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 声明launch参数
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    # 配置文件路径
    config_file_path = PathJoinSubstitution([
        FindPackageShare('rosiwit_slam'),
        'config',
        'livox_avia.yaml'
    ])

    # 创建节点
    fast_lio2_node = Node(
        package='rosiwit_slam',
        executable='fast_lio2_node',
        name='fast_lio2_node',
        output='screen',
        parameters=[
            config_file_path,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        fast_lio2_node
    ])