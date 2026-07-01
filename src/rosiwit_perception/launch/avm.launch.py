from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['ros2', 'run', 'rosiwit_perception', 'avm_node'],
            output='screen',
        ),
    ])
