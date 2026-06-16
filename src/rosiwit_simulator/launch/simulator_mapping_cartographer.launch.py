import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    cartographer_ros_dir = get_package_share_directory('cartographer_ros')

    # cartographer_node
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        arguments=[
            '-configuration_directory', os.path.join(cartographer_ros_dir, 'configuration_files'),
            '-configuration_basename', 'test.lua',
        ],
        output='screen',
        remappings=[
            ('scan', '/scan'),
        ]
    )

    # cartographer_occupancy_grid_node
    cartographer_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        arguments=['-resolution', '0.05'],
    )

    return LaunchDescription([
        cartographer_node,
        cartographer_occupancy_grid_node,
    ])
