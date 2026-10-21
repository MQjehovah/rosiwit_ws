"""Launch QR detector node.

Usage:
    ros2 launch rosiwit_perception_ros2 qr_detector.launch.py
    ros2 launch rosiwit_perception_ros2 qr_detector.launch.py \
        image_topic:=/camera/image_raw camera_info_topic:=/camera/camera_info
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('rosiwit_perception_ros2')
    default_params = os.path.join(pkg_share, 'config', 'qr_detector.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'image_topic',
            default_value='/camera_front/camera/image_raw',
            description='sensor_msgs/Image topic to subscribe'),
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value='/camera_front/camera/camera_info',
            description='sensor_msgs/CameraInfo topic for pose estimation'),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Parameter YAML file'),
        Node(
            package='rosiwit_perception_ros2',
            executable='qr_detector_node_wrapper',
            name='qr_detector',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {
                    'image_topic': LaunchConfiguration('image_topic'),
                    'camera_info_topic': LaunchConfiguration('camera_info_topic'),
                },
            ],
        ),
    ])
