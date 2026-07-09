# ============================================================
# Diffbot Navigation - 导航启动文件
# ============================================================

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 获取包路径
    pkg_share = FindPackageShare('rosiwit_navigation')
    pkg_dir = pkg_share.find('rosiwit_navigation')

    # 声明启动参数
    declared_arguments = [
        # 是否使用仿真时间
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        # 地图文件路径
        DeclareLaunchArgument(
            'map',
            default_value='',
            description='Full path to map yaml file to load'
        ),
        # 参数文件路径
        DeclareLaunchArgument(
            'params_file',
            default_value='',
            description='Full path to the ROS2 parameters file to use for all launched nodes'
        ),
        # 是否启动RViz
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Whether to start RViz'
        ),
        # RViz配置文件
        DeclareLaunchArgument(
            'rviz_config_file',
            default_value='',
            description='Full path to the RViz config file to use'
        ),
        # 自动启动
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically startup the nav2 stack'
        ),
        # 是否使用Nav2
        DeclareLaunchArgument(
            'use_nav2',
            default_value='true',
            description='Whether to use Nav2 stack'
        ),
    ]

    # 获取参数配置
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    use_rviz = LaunchConfiguration('use_rviz')
    autostart = LaunchConfiguration('autostart')

    # ============================================================
    # 自定义导航节点
    # ============================================================
    navigation_nodes = [
        # 平滑导航节点（核心导航控制）
        Node(
            package='rosiwit_navigation',
            executable='smooth_navigation_node',
            name='smooth_navigation',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                params_file,
            ],
            remappings=[
                ('/cmd_vel', '/cmd_vel'),
                ('/odom', '/odom'),
                ('/map', '/map'),
            ],
        ),
    ]

    # ============================================================
    # Nav2 核心组件（可选）
    # ============================================================
    nav2_nodes = [
        # Controller Server
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[params_file],
            condition=IfCondition(LaunchConfiguration('use_nav2')),
        ),
        # Planner Server
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[params_file],
            condition=IfCondition(LaunchConfiguration('use_nav2')),
        ),
        # Behavior Server
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[params_file],
            condition=IfCondition(LaunchConfiguration('use_nav2')),
        ),
        # BT Navigator
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[params_file],
            condition=IfCondition(LaunchConfiguration('use_nav2')),
        ),
    ]

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

    # ============================================================
    # 组合所有节点
    # ============================================================
    return LaunchDescription(
        declared_arguments +
        navigation_nodes +
        nav2_nodes +
        [rviz_node]
    )

    # RViz节点 (占位)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config_file')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        output='screen'
    )

    return LaunchDescription(declared_arguments + [
        # 提示信息
        # 注意: 完整的导航节点将在源代码实现后添加
        # 目前仅为占位配置

        # RViz可视化
        rviz_node,
    ])