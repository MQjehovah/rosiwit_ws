# FAST-LIVO2 专用入口: 只是把标准节点 rosiwit_slam_ros2 的默认配置换成 LIVO 那份.
#
#   ros2 launch rosiwit_slam_ros2 fast_livo2.launch.py
#   # 等价于
#   ros2 launch rosiwit_slam_ros2 slam.launch.py config:=fast_livo2_ntu.yaml use_sim_time:=true
#
# 节点内部仍是 SlamPipeline + fast_livo2_frontend (见 config 里 global.slam_algorithm /
# global.modules.frontend), 没有单独的 LIVO 节点.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg = get_package_share_directory('rosiwit_slam_ros2')

    config_arg = DeclareLaunchArgument(
        'config', default_value='fast_livo2_ntu.yaml',
        description='Config file name under <pkg>/config (LIVO: fast_livo2_ntu.yaml)')
    sim_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    rviz_arg = DeclareLaunchArgument('use_rviz', default_value='true')
    algo_arg = DeclareLaunchArgument('slam_algorithm', default_value='fast_lio2_pipeline')
    mt_arg = DeclareLaunchArgument('use_multi_threaded', default_value='true')

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'slam.launch.py')),
        launch_arguments={
            'config': LaunchConfiguration('config'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'use_rviz': LaunchConfiguration('use_rviz'),
            'slam_algorithm': LaunchConfiguration('slam_algorithm'),
            'use_multi_threaded': LaunchConfiguration('use_multi_threaded'),
        }.items(),
    )

    return LaunchDescription([config_arg, sim_arg, rviz_arg, algo_arg, mt_arg, slam])
