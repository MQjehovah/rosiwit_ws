import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro


def generate_launch_description():
    simulator_dir = get_package_share_directory('rosiwit_simulator')
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')

    # 设置 Gazebo 模型路径
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=os.path.join(simulator_dir, 'models')
    )

    # Launch Arguments
    world_name_arg = DeclareLaunchArgument(
        'world_name',
        default_value=os.path.join(simulator_dir, 'world', 'house.world')
    )
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    gui_arg = DeclareLaunchArgument('gui', default_value='true')
    paused_arg = DeclareLaunchArgument('paused', default_value='false')
    headless_arg = DeclareLaunchArgument('headless', default_value='false')
    debug_arg = DeclareLaunchArgument('debug', default_value='false')

    # 运行 Gazebo 仿真环境
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

    # 加载 3D 雷达机器人模型描述参数 (xacro -> URDF)
    xacro_file = os.path.join(simulator_dir, 'urdf', 'xacro', 'gazebo', 'mbot_with_lidar3d_gazebo.xacro')
    robot_description_content = xacro.process_file(xacro_file).toxml()

    # Spawn 模型 (ROS2: spawn_entity.py)
    spawn_model = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-urdf', '-model', 'mrobot',
            '-param', 'robot_description'
        ],
        output='screen'
    )

    # joint_state_publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    # robot_state_publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'publish_frequency': 50.0,
        }]
    )

    # RViz2
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', os.path.join(simulator_dir, 'rviz', 'simulator_3d.rviz')],
        output='screen'
    )

    return LaunchDescription([
        set_gazebo_model_path,
        world_name_arg,
        use_sim_time_arg,
        gui_arg,
        paused_arg,
        headless_arg,
        debug_arg,
        gazebo,
        spawn_model,
        joint_state_publisher,
        robot_state_publisher,
        rviz,
    ])
