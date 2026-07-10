# ============================================================
# Diffbot Navigation - 导航启动文件
# ============================================================

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 声明启动参数
    declared_arguments = [
        # 是否使用仿真时间
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        # 地图文件
        DeclareLaunchArgument(
            'map',
            default_value='',
            description='Full path to map yaml file to load'
        ),
        # 是否启动RViz
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Whether to start RViz'
        ),
        # RViz配置文件
        DeclareLaunchArgument(
            'rviz_config_file',
            default_value='',
            description='Full path to the RViz config file to use'
        ),
    ]

    # 获取参数配置
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file = LaunchConfiguration('map')
    use_rviz = LaunchConfiguration('use_rviz')

    # ============================================================
    # 自定义导航节点
    # ============================================================
    navigation_nodes = [
        # 平滑导航节点（核心导航控制）
        Node(
            package='rosiwit_navigation',
            executable='rosiwit_navigation_node',
            name='rosiwit_navigation_node',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
            ],
            remappings=[
                ('/cmd_vel', '/cmd_vel'),
                ('/odom', '/odom'),
                ('/map', '/map'),
            ],
        ),
    ]

    # ============================================================
    # 地图服务 (nav2_map_server)
    # ============================================================
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_file, 'use_sim_time': use_sim_time}],
    )

    # Auto-activate map_server lifecycle
    map_configure = TimerAction(period=1.0, actions=[ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set', '/map_server', 'configure'])])
    map_activate = TimerAction(period=2.0, actions=[ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set', '/map_server', 'activate'])])

    # ============================================================
    # RViz可视化
    # ============================================================
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config_file')],
        parameters=[
            {'use_sim_time': use_sim_time},
        ],
        condition=IfCondition(use_rviz),
    )

    # Auto-activate rosiwit_navigation_node lifecycle (2.5s after launch)
    nav_configure = TimerAction(period=2.5, actions=[ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set', '/rosiwit_navigation_node', 'configure'])])
    nav_activate = TimerAction(period=4.0, actions=[ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set', '/rosiwit_navigation_node', 'activate'])])

    # ============================================================
    # 组合所有节点
    # ============================================================
    return LaunchDescription(
        declared_arguments +
        navigation_nodes +
        [map_server, map_configure, map_activate] +
        [nav_configure, nav_activate] +
        [rviz_node]
    )

