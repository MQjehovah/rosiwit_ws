"""
仿真 SLAM 建图 - Gazebo 版
使用 Gazebo 物理引擎模拟真实传感器数据驱动 rosiwit_slam 建图

启动内容:
1. Gazebo 仿真环境 (house.world) + 机器人模型 (mbot + VLP-16 + IMU)
2. rosiwit_slam (FAST-LIO2) 建图节点
3. 自动驾驶脚本 (figure8 轨迹) 让机器人运动探索环境
4. 延迟自动保存地图

话题:
  /velodyne_points  - 3D 点云 (来自 Gazebo VLP-16 插件)
  /imu              - IMU 数据 (来自 Gazebo IMU 插件)
  /cmd_vel          - 速度控制 (自动驾驶脚本发布)
  /odom             - 里程计 (来自 Gazebo 差速驱动插件)
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    TimerAction,
    ExecuteProcess,
    SetEnvironmentVariable,
    IncludeLaunchDescription,
)
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ==================== 参数 ====================
    map_output_dir_arg = DeclareLaunchArgument(
        'map_output_dir', default_value='/tmp/rosiwit_slam_map',
        description='Directory to save the output map'
    )
    slam_config_arg = DeclareLaunchArgument(
        'slam_config', default_value='velodyne_vlp16.yaml',
        description='SLAM configuration file'
    )
    gui_arg = DeclareLaunchArgument(
        'gui', default_value='true',
        description='Whether to show Gazebo GUI'
    )
    headless_arg = DeclareLaunchArgument(
        'headless', default_value='false',
        description='Run Gazebo in headless mode (no GUI)'
    )
    drive_mode_arg = DeclareLaunchArgument(
        'drive_mode', default_value='figure8',
        description='Auto-drive mode: figure8, circle, square, explore'
    )
    drive_duration_arg = DeclareLaunchArgument(
        'drive_duration', default_value='120.0',
        description='Auto-drive duration in seconds'
    )

    # ==================== 1. Gazebo 环境 + 机器人模型 ====================

    # 设置 Gazebo 模型路径
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=PathJoinSubstitution([
            FindPackageShare('rosiwit_simulator'), 'models'
        ])
    )

    # 启动 Gazebo
    world_file = PathJoinSubstitution([
        FindPackageShare('rosiwit_simulator'), 'world', 'house.world'
    ])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'
            ])
        ),
        launch_arguments={
            'world': world_file,
            'gui': LaunchConfiguration('gui'),
            'paused': 'false',
            'use_sim_time': 'true',
            'headless': LaunchConfiguration('headless'),
            'debug': 'false',
        }.items()
    )

    # 使用 Command substitution 处理 xacro (避免 import xacro)
    xacro_file = PathJoinSubstitution([
        FindPackageShare('rosiwit_simulator'),
        'urdf', 'xacro', 'gazebo', 'mbot_with_lidar3d_gazebo.xacro'
    ])
    robot_description_content = Command([
        'xacro ', xacro_file
    ])

    # robot_state_publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(robot_description_content, value_type=str),
            'use_sim_time': True,
            'publish_frequency': 50.0,
        }],
    )

    # Spawn 机器人模型到 Gazebo (通过 /robot_description 话题获取 URDF)
    spawn_model = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'mrobot',
            '-x', '0', '-y', '0', '-z', '0.1',
        ],
        output='screen',
    )

    # ==================== 2. 自动驾驶脚本 (让机器人运动) ====================
    auto_drive_node = Node(
        package='rosiwit_simulator',
        executable='sim_drive.py',
        name='figure8_driver',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'linear_speed': 0.3,
            'angular_speed': 0.5,
            'mode': LaunchConfiguration('drive_mode'),
            'duration': LaunchConfiguration('drive_duration'),
        }],
    )

    # ==================== 3. rosiwit_slam 节点 ====================
    slam_config_path = PathJoinSubstitution([
        FindPackageShare('rosiwit_slam'), 'config', 'velodyne_vlp16.yaml'
    ])

    slam_node = Node(
        package='rosiwit_slam',
        executable='fast_lio2_node',
        name='fast_lio2_node',
        output='screen',
        parameters=[{
            'config_file': slam_config_path,
            'use_sim_time': True,
            'lidar_topic': '/velodyne_points',
            'imu_topic': '/imu',
            'map_output_dir': LaunchConfiguration('map_output_dir'),
        }],
    )
    # 延迟 5 秒启动 SLAM，等 Gazebo 和传感器数据稳定
    slam_delayed = TimerAction(period=5.0, actions=[slam_node])

    # ==================== 4. 地图保存 (延迟后自动保存) ====================
    save_map_cmd = ExecuteProcess(
        cmd=[
            'bash', '-c',
            'echo "=== Waiting for SLAM to build map (60s) ===" && '
            'sleep 60 && '
            'echo "=== Saving map ===" && '
            'ros2 service call /save_map std_srvs/srv/Trigger && '
            'echo "=== Map saved ===" && '
            'sleep 5 && '
            'ros2 service call /save_pcd std_srvs/srv/Trigger && '
            'echo "=== PCD saved ==="',
        ],
        output='screen',
    )

    return LaunchDescription([
        # 参数声明
        map_output_dir_arg,
        slam_config_arg,
        gui_arg,
        headless_arg,
        drive_mode_arg,
        drive_duration_arg,

        # 环境设置
        set_gazebo_model_path,

        # Gazebo + 机器人
        gazebo,
        robot_state_publisher,
        spawn_model,

        # 自动驾驶
        auto_drive_node,

        # SLAM
        slam_delayed,

        # 地图保存
        save_map_cmd,
    ])
