"""
sim_slam_nav.launch.py - 仿真环境完整系统启动
组合启动: Simulator + SLAM + Navigation + App
这是最常用的启动文件，用于仿真测试

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
    # 参数声明
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true')
    declare_map_path = DeclareLaunchArgument(
        'map_path', default_value='/tmp/rosiwit_sim_map')
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='false')
    declare_use_gazebo = DeclareLaunchArgument(
        'use_gazebo', default_value='true',
        description='Use Gazebo (true) or Python script (false) for simulation')
    declare_gui = DeclareLaunchArgument(
        'gui', default_value='true',
        description='Show Gazebo GUI')

    # 环境变量
    set_env = SetEnvironmentVariable(
        'RCUTILS_LOGGING_USE_ROSOUT', '0')

    # ========== 动态启动逻辑 ==========
    def launch_setup(context):
        use_gazebo_str = context.launch_configurations.get('use_gazebo', 'true')
        use_sim_time_str = context.launch_configurations.get('use_sim_time', 'true')
        map_path_str = context.launch_configurations.get('map_path', '/tmp/rosiwit_sim_map')
        gui_str = context.launch_configurations.get('gui', 'true')
        use_sim_time_bool = use_sim_time_str == 'true'

        actions = []

        # ========== 1. 模拟器 ==========
        if use_gazebo_str == 'true':
            # --- Gazebo 模式 ---
            actions.append(LogInfo(msg='[sim_slam_nav] Starting Gazebo simulator...'))

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

            # 加载机器人 URDF (使用 Command substitution 处理 xacro)
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
            nav_delay = 11.0
            app_delay = 15.0
        else:
            # --- Python 脚本模式 (原始方式) ---
            actions.append(LogInfo(msg='[sim_slam_nav] Starting Python script simulator...'))

            actions.append(Node(
                package='rosiwit_simulator',
                executable='simulated_sensor.py',
                name='simulated_sensor',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time_bool,
                }],
            ))

            slam_delay = 3.0
            nav_delay = 6.0
            app_delay = 10.0

        # ========== 2. SLAM ==========
        slam_config_path = os.path.join(
            get_package_share_directory('rosiwit_slam'), 'config', 'velodyne_vlp16.yaml'
        )

        slam = Node(
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
        actions.append(TimerAction(period=slam_delay, actions=[slam]))

        # ========== 3. Map Server ==========
        map_server = Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time_bool,
                'yaml_filename': os.path.join(map_path_str, 'map.yaml'),
            }],
        )
        lifecycle = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time_bool,
                'autostart': True,
                'node_names': ['map_server'],
            }],
        )
        actions.append(TimerAction(period=slam_delay + 1.0, actions=[map_server, lifecycle]))

        # ========== 4. Navigation (Nav2) ==========
        nav2_config = PathJoinSubstitution([
            FindPackageShare('rosiwit_simulator'), 'config', 'diff', 'nav2_params.yaml'
        ])
        nav2_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py'
                ])
            ),
            launch_arguments={
                'use_sim_time': use_sim_time_str,
                'params_file': nav2_config,
            }.items()
        )
        actions.append(TimerAction(period=nav_delay, actions=[nav2_launch]))

        # ========== 5. App ==========
        app = Node(
            package='rosiwit_app',
            executable='app_node',
            name='rosiwit_app',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time_bool,
                'waypoints_file': '',
                'map_path': map_path_str,
            }],
        )
        actions.append(TimerAction(period=app_delay, actions=[app]))

        # ========== 6. RViz (可选) ==========
        rviz_config = PathJoinSubstitution([
            FindPackageShare('rosiwit_app'), 'rviz', 'rosiwit_app.rviz'
        ])
        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time_bool}],
        )
        actions.append(TimerAction(period=2.0, actions=[rviz_node]))

        mode_name = 'Gazebo' if use_gazebo_str == 'true' else 'Python Script'
        actions.insert(0, LogInfo(msg=f'[sim_slam_nav] Mode: {mode_name}'))
        actions.insert(1, LogInfo(msg='[sim_slam_nav] All subsystems launching with staggered delays...'))

        return actions

    return LaunchDescription([
        declare_use_sim_time,
        declare_map_path,
        declare_use_rviz,
        declare_use_gazebo,
        declare_gui,
        set_env,

        OpaqueFunction(function=launch_setup),
    ])
