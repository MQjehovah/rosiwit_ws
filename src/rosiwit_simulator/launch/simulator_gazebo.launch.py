import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, OpaqueFunction, TimerAction, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    simulator_dir = get_package_share_directory('rosiwit_simulator')
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')

    os.environ['LIBGL_ALWAYS_SOFTWARE'] = '0'
    os.environ['GALLIUM_DRIVER'] = 'd3d12'

    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=os.path.join(simulator_dir, 'models')
    )

    # 参数
    world_name_arg = DeclareLaunchArgument(
        'world_name',
        default_value=os.path.join(simulator_dir, 'world', 'house.world')
    )
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    gui_arg = DeclareLaunchArgument('gui', default_value='true')
    paused_arg = DeclareLaunchArgument('paused', default_value='false')
    headless_arg = DeclareLaunchArgument('headless', default_value='false')
    debug_arg = DeclareLaunchArgument('debug', default_value='false')
    use_3d_lidar_arg = DeclareLaunchArgument('use_3d_lidar', default_value='false')
    run_slam_arg = DeclareLaunchArgument('run_slam', default_value='false')
    drive_mode_arg = DeclareLaunchArgument('drive_mode', default_value='explore')
    drive_duration_arg = DeclareLaunchArgument('drive_duration', default_value='180.0')
    enable_backend_arg = DeclareLaunchArgument('enable_backend', default_value='true')
    backend_type_arg = DeclareLaunchArgument('backend_type', default_value='ceres_pose_graph')

    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': LaunchConfiguration('world_name'),
            'debug': LaunchConfiguration('debug'),
            'gui': LaunchConfiguration('gui'),
            'paused': LaunchConfiguration('paused'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'headless': LaunchConfiguration('headless'),
        }.items()
    )

    # 机器人模型
    def launch_robot(context):
        use_3d = context.launch_configurations.get('use_3d_lidar', 'false')
        xacro_name = 'mbot_with_lidar3d_gazebo.xacro' if use_3d == 'true' else 'mbot_with_fisheye_gazebo.xacro'
        xacro_path = os.path.join(simulator_dir, 'urdf', 'xacro', 'gazebo', xacro_name)
        robot_desc = Command(['xacro ', xacro_path])
        return [
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                output='screen',
                parameters=[{
                    'robot_description': ParameterValue(robot_desc, value_type=str),
                    'publish_frequency': 50.0,
                }]
            ),
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=['-topic', 'robot_description', '-entity', 'mrobot', '-timeout', '60'],
                output='screen'
            ),
        ]

    # rosiwit_slam
    slam_config_path = os.path.join(
        get_package_share_directory('rosiwit_slam'), 'config', 'sim_vlp16.yaml'
    )

    slam_node = Node(
        package='rosiwit_slam',
        executable='rosiwit_slam',
        name='rosiwit_slam',
        output='screen',
        parameters=[{
            'config_path': slam_config_path,
            'use_sim_time': True,
            'lidar_topic': '/velodyne_points',
            'imu_topic': '/imu',
            'slam_algorithm': 'fast_lio2_pipeline',
        }],
    )
    slam_delayed = TimerAction(
        period=3.0,
        actions=[slam_node],
        condition=IfCondition(LaunchConfiguration('run_slam'))
    )

    # 自动驾驶
    auto_drive = Node(
        package='rosiwit_simulator',
        executable='sim_drive.py',
        name='auto_driver',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'linear_speed': 0.5,
            'angular_speed': 0.3,
            'mode': LaunchConfiguration('drive_mode'),
            'duration': LaunchConfiguration('drive_duration'),
        }],
        condition=IfCondition(LaunchConfiguration('run_slam'))
    )

    # 地图保存
    save_map = ExecuteProcess(
        cmd=['bash', '-c',
             'sleep 180 && '
             'ros2 service call /save_map rosiwit_slam/srv/SaveMap '
             '"{path: /tmp/large_loop_map.pcd}" && '
             'ros2 service call /save_grid_map rosiwit_slam/srv/SaveGridMap '
             '"{pgm_path: /tmp/large_loop_grid.pgm, yaml_path: /tmp/large_loop_grid.yaml, resolution: 0.05}" || '
             'echo "map save done"',
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('run_slam'))
    )

    return LaunchDescription([
        set_gazebo_model_path,
        world_name_arg, use_sim_time_arg, gui_arg, paused_arg, headless_arg, debug_arg,
        use_3d_lidar_arg, run_slam_arg, drive_mode_arg, drive_duration_arg,
        enable_backend_arg, backend_type_arg,
        gazebo,
        OpaqueFunction(function=launch_robot),
        slam_delayed,
        auto_drive,
        save_map,
    ])
