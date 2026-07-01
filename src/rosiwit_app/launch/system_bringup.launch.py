"""
system_bringup.launch.py - 一键启动完整移动机器人系统
启动顺序: Simulator → SLAM → MapServer(可选) → Navigation → App

支持两种模拟模式:
  use_gazebo=true:  使用 Gazebo 物理引擎 (真实传感器仿真)
  use_gazebo=false: 使用 Python 脚本模拟 (轻量级，无需 GPU)
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    TimerAction, LogInfo, OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.substitutions import (
    LaunchConfiguration, Command, PathJoinSubstitution
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ==================== 参数声明 ====================
    declare_use_simulator = DeclareLaunchArgument(
        'use_simulator', default_value='true',
        description='Whether to start the simulator')

    declare_use_gazebo = DeclareLaunchArgument(
        'use_gazebo', default_value='true',
        description='Use Gazebo (true) or Python script (false) for simulation')

    declare_use_navigation = DeclareLaunchArgument(
        'use_navigation', default_value='true',
        description='Whether to start navigation stack')

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='false',
        description='Whether to start RViz')

    declare_map_path = DeclareLaunchArgument(
        'map_path', default_value='/tmp/rosiwit_sim_map',
        description='Directory for map files')

    declare_slam_config = DeclareLaunchArgument(
        'slam_config', default_value='velodyne_vlp16.yaml',
        description='SLAM configuration file')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation time')

    declare_gui = DeclareLaunchArgument(
        'gui', default_value='true',
        description='Show Gazebo GUI (only for Gazebo mode)')

    # ==================== 动态启动逻辑 ====================
    def launch_setup(context):
        use_simulator_str = context.launch_configurations.get('use_simulator', 'true')
        use_gazebo_str = context.launch_configurations.get('use_gazebo', 'true')
        use_navigation_str = context.launch_configurations.get('use_navigation', 'true')
        use_sim_time_str = context.launch_configurations.get('use_sim_time', 'true')
        map_path_str = context.launch_configurations.get('map_path', '/tmp/rosiwit_sim_map')
        slam_config_str = context.launch_configurations.get('slam_config', 'velodyne_vlp16.yaml')
        gui_str = context.launch_configurations.get('gui', 'true')

        use_sim_time_bool = use_sim_time_str == 'true'

        actions = []

        # ========== 1. 启动模拟器 ==========
        if use_simulator_str == 'true':
            if use_gazebo_str == 'true':
                # --- Gazebo 模式 ---
                actions.append(LogInfo(msg='[system_bringup] Starting Gazebo simulator...'))

                simulator_dir = get_package_share_directory('rosiwit_simulator')
                gazebo_ros_dir = get_package_share_directory('gazebo_ros')

                # 设置 Gazebo 模型路径
                actions.append(SetEnvironmentVariable(
                    name='GAZEBO_MODEL_PATH',
                    value=os.path.join(simulator_dir, 'models')
                ))

                # 启动 Gazebo
                actions.append(IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(gazebo_ros_dir, 'launch', 'gazebo.launch.py')
                    ),
                    launch_arguments={
                        'world': os.path.join(simulator_dir, 'world', 'house.world'),
                        'gui': gui_str,
                        'paused': 'false',
                        'use_sim_time': 'true',
                        'debug': 'false',
                    }.items()
                ))

                # 使用 Command 处理 xacro
                xacro_file = os.path.join(
                    simulator_dir, 'urdf', 'xacro', 'gazebo', 'mbot_with_lidar3d_gazebo.xacro'
                )
                robot_description_content = Command(['xacro ', xacro_file])

                actions.append(Node(
                    package='robot_state_publisher',
                    executable='robot_state_publisher',
                    name='robot_state_publisher',
                    output='screen',
                    parameters=[{
                        'robot_description': ParameterValue(robot_description_content, value_type=str),
                        'use_sim_time': True,
                        'publish_frequency': 50.0,
                    }],
                ))

                # Spawn 机器人 (通过 /robot_description 话题获取 URDF)
                actions.append(Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    arguments=[
                        '-topic', 'robot_description',
                        '-entity', 'mrobot',
                        '-x', '0', '-y', '0', '-z', '0.1',
                    ],
                    output='screen',
                ))

                # 自动驾驶脚本
                actions.append(Node(
                    package='rosiwit_simulator',
                    executable='sim_drive.py',
                    name='figure8_driver',
                    output='screen',
                    parameters=[{
                        'use_sim_time': True,
                        'linear_speed': 0.3,
                        'angular_speed': 0.5,
                        'mode': 'figure8',
                        'duration': 120.0,
                    }],
                ))

                slam_delay = 8.0

            else:
                # --- Python 脚本模式 ---
                actions.append(LogInfo(msg='[system_bringup] Starting Python script simulator...'))
                actions.append(Node(
                    package='rosiwit_simulator',
                    executable='simulated_sensor.py',
                    name='simulated_sensor',
                    output='screen',
                    parameters=[{
                        'use_sim_time': use_sim_time_bool,
                        'room_size_x': 10.0,
                        'room_size_y': 8.0,
                        'room_size_z': 3.0,
                    }],
                ))
                slam_delay = 3.0
        else:
            actions.append(LogInfo(msg='[system_bringup] Simulator disabled'))
            slam_delay = 0.0

        # ========== 2. SLAM ==========
        slam_dir = get_package_share_directory('rosiwit_slam')
        slam_config_path = os.path.join(slam_dir, 'config', slam_config_str)

        slam_node = Node(
            package='rosiwit_slam',
            executable='fast_lio2_node',
            name='fast_lio2',
            output='screen',
            parameters=[{
                'config_file': slam_config_path,
                'use_sim_time': use_sim_time_bool,
                'map.map_path': map_path_str,
            }],
        )
        actions.append(TimerAction(period=slam_delay, actions=[slam_node]))

        # ========== 3. Navigation ==========
        if use_navigation_str == 'true':
            def create_nav_nodes(nav_context):
                nav_actions = []

                nav_actions.append(Node(
                    package='nav2_map_server',
                    executable='map_server',
                    name='map_server',
                    output='screen',
                    parameters=[{
                        'use_sim_time': use_sim_time_bool,
                        'yaml_filename': os.path.join(map_path_str, 'map.yaml'),
                    }],
                ))

                nav_actions.append(Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_navigation',
                    output='screen',
                    parameters=[{
                        'use_sim_time': use_sim_time_bool,
                        'autostart': True,
                        'node_names': ['map_server'],
                    }],
                ))

                return nav_actions

            nav_delayed = TimerAction(
                period=slam_delay + 3.0,
                actions=[OpaqueFunction(function=create_nav_nodes)]
            )
            actions.append(nav_delayed)

        # ========== 4. App ==========
        app_node = Node(
            package='rosiwit_app',
            executable='app_node',
            name='rosiwit_app',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time_bool,
                'waypoints_file': '',
            }],
        )
        actions.append(TimerAction(period=slam_delay + 8.0, actions=[app_node]))

        actions.append(LogInfo(msg='[rosiwit_app] All subsystems launching with staggered delays...'))

        return actions

    # ==================== 组装 ====================
    return LaunchDescription([
        # 参数声明
        declare_use_simulator,
        declare_use_gazebo,
        declare_use_navigation,
        declare_use_rviz,
        declare_map_path,
        declare_slam_config,
        declare_use_sim_time,
        declare_gui,

        # 动态启动逻辑
        OpaqueFunction(function=launch_setup),
    ])
