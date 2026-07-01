"""
仿真 SLAM 建图集成 Launch 文件
同时启动:
1. Gazebo 仿真环境 (headless)
2. 机器人模型 (3D LiDAR + IMU)
3. rosiwit_slam 建图节点
4. 自动运动节点 (8字形轨迹)
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro


def generate_launch_description():
    simulator_dir = get_package_share_directory('rosiwit_simulator')
    slam_dir = get_package_share_directory('rosiwit_slam')
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')

    # ==================== 环境变量 ====================
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=os.path.join(simulator_dir, 'models')
    )

    # ==================== Launch 参数 ====================
    world_name_arg = DeclareLaunchArgument(
        'world_name',
        default_value=os.path.join(simulator_dir, 'world', 'house.world'),
        description='Gazebo world file'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation time'
    )
    slam_config_arg = DeclareLaunchArgument(
        'slam_config', default_value='velodyne_vlp16.yaml',
        description='SLAM configuration file'
    )
    map_output_dir_arg = DeclareLaunchArgument(
        'map_output_dir', default_value='/tmp/rosiwit_slam_map',
        description='Directory to save the output map'
    )
    auto_drive_arg = DeclareLaunchArgument(
        'auto_drive', default_value='true',
        description='Enable automatic robot movement'
    )

    # ==================== 1. Gazebo (headless) ====================
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': LaunchConfiguration('world_name'),
            'gui': 'false',
            'paused': 'false',
            'use_sim_time': 'true',
            'verbose': 'true',
        }.items()
    )

    # ==================== 2. 机器人模型 ====================
    xacro_file = os.path.join(
        simulator_dir, 'urdf', 'xacro', 'gazebo', 'mbot_with_lidar3d_gazebo.xacro'
    )
    robot_description_content = xacro.process_file(xacro_file).toxml()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': True,
        }]
    )

    # ==================== 3. Spawn 机器人 ====================
    spawn_model = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-urdf', '-model', 'mrobot',
            '-param', 'robot_description',
            '-x', '0', '-y', '0', '-z', '0.1',
        ],
        output='screen'
    )

    # ==================== 4. rosiwit_slam 节点 ====================
    slam_node = Node(
        package='rosiwit_slam',
        executable='fast_lio2_node',
        name='fast_lio2_node',
        output='screen',
        parameters=[{
            'config_file': os.path.join(slam_dir, 'config', 'velodyne_vlp16.yaml'),
            'use_sim_time': True,
            'lidar_topic': '/velodyne_points',
            'imu_topic': '/imu',
            'map_output_dir': LaunchConfiguration('map_output_dir'),
        }]
    )

    # ==================== 5. 自动运动脚本 ====================
    # 发布 /cmd_vel 让机器人走 8 字形轨迹，持续产生运动数据供 SLAM 建图
    auto_drive_script = os.path.join(
        simulator_dir, 'launch', 'auto_drive_figure8.py'
    )
    auto_drive_node = Node(
        package='rosiwit_simulator',
        executable='auto_drive_figure8.py',
        name='auto_drive',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'linear_speed': 0.3,
            'angular_speed': 0.5,
        }]
    )

    # ==================== 组装 Launch ====================
    return LaunchDescription([
        # 环境变量
        set_gazebo_model_path,
        # 参数声明
        world_name_arg,
        use_sim_time_arg,
        slam_config_arg,
        map_output_dir_arg,
        auto_drive_arg,
        # Gazebo 立即启动
        gazebo,
        robot_state_publisher,
        spawn_model,
        # SLAM 延迟 5 秒启动，等 Gazebo 和 spawn 完成
        TimerAction(period=5.0, actions=[slam_node]),
        # 自动运动延迟 8 秒启动，等 SLAM 初始化完成
        TimerAction(period=8.0, actions=[auto_drive_node]),
    ])
