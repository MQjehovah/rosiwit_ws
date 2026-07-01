from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rosiwit_perception',
            executable='avm_node',
            name='avm_node',
            output='screen',
        ),
    ])
