#!/usr/bin/python3
# -- coding: utf-8 -*-
"""Generic launch: any sensor_msgs/PointCloud2 lidar + IMU (+ optional camera).

The lidar point layout is auto-detected by preprocess.lidar_type=8 (GENERIC):
  x/y/z + optional intensity + optional per-point time ("time"/"timestamp"/"t"/...).

Example:
  ros2 launch fast_livo mapping_generic.launch.py \
      params_file:=/path/to/vanjee.yaml camera_params_file:=/path/to/camera.yaml \
      use_sim_time:=True
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory("fast_livo"), "config")
    rviz_config_file = os.path.join(
        get_package_share_directory("fast_livo"), "rviz_cfg", "fast_livo2.rviz")

    params_arg = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(config_dir, "vanjee.yaml"),
        description="LiDAR/IMU/algorithm parameter file",
    )
    camera_params_arg = DeclareLaunchArgument(
        "camera_params_file",
        default_value=os.path.join(config_dir, "camera_pinhole.yaml"),
        description="Camera intrinsics file (required even when img_en: 0)",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Use the bag /clock instead of wall time (set true when playing a rosbag)",
    )
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="False",
        description="Start RViz2 with the fast_livo2 config",
    )

    return LaunchDescription([
        params_arg,
        camera_params_arg,
        use_sim_time_arg,
        use_rviz_arg,
        Node(
            package="fast_livo",
            executable="fastlivo_mapping",
            name="laserMapping",
            parameters=[
                LaunchConfiguration("params_file"),
                LaunchConfiguration("camera_params_file"),
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
            ],
            output="screen",
        ),
        Node(
            condition=IfCondition(LaunchConfiguration("use_rviz")),
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config_file],
            output="screen",
        ),
    ])
