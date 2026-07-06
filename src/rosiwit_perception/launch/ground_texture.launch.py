"""Launch AVM + ground texture mapper alongside FAST-LIO2."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rosiwit_perception',
            executable='avm_node_wrapper',
            name='avm_node',
            output='screen',
        ),
        Node(
            package='rosiwit_perception',
            executable='ground_texture_mapper_wrapper',
            name='ground_texture_mapper',
            output='screen',
        ),
    ])
