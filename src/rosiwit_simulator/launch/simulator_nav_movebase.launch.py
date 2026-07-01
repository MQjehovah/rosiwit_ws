import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    simulator_dir = get_package_share_directory('rosiwit_simulator')

    # Map file argument
    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value=os.path.join(simulator_dir, 'map', 'test.yaml')
    )

    # Initial pose arguments
    initial_pose_x_arg = DeclareLaunchArgument('initial_pose_x', default_value='0.0')
    initial_pose_y_arg = DeclareLaunchArgument('initial_pose_y', default_value='0.0')
    initial_pose_a_arg = DeclareLaunchArgument('initial_pose_a', default_value='0.0')

    # Map server 节点
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        arguments=[LaunchConfiguration('map_file')],
    )

    # AMCL (include amcl_diff)
    amcl_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(simulator_dir, 'launch', 'include', 'amcl_diff.launch.py')
        ),
        launch_arguments={
            'initial_pose_x': LaunchConfiguration('initial_pose_x'),
            'initial_pose_y': LaunchConfiguration('initial_pose_y'),
            'initial_pose_a': LaunchConfiguration('initial_pose_a'),
        }.items()
    )

    # TEB move_base (include teb_move_base_diff)
    teb_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(simulator_dir, 'launch', 'include', 'teb_move_base_diff.launch.py')
        )
    )

    return LaunchDescription([
        map_file_arg,
        initial_pose_x_arg,
        initial_pose_y_arg,
        initial_pose_a_arg,
        map_server,
        amcl_include,
        teb_include,
    ])
