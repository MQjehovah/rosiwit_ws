import os
import tempfile
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro


def generate_launch_description():
    simulator_dir = get_package_share_directory('rosiwit_simulator')
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')

    os.environ['LIBGL_ALWAYS_SOFTWARE'] = '0'
    os.environ['GALLIUM_DRIVER'] = 'd3d12'

    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=os.path.join(simulator_dir, 'models')
    )

    world_name_arg = DeclareLaunchArgument(
        'world_name',
        default_value=os.path.join(simulator_dir, 'world', 'large.world')
    )
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    gui_arg = DeclareLaunchArgument('gui', default_value='true')
    paused_arg = DeclareLaunchArgument('paused', default_value='false')
    headless_arg = DeclareLaunchArgument('headless', default_value='false')
    debug_arg = DeclareLaunchArgument('debug', default_value='false')
    use_3d_lidar_arg = DeclareLaunchArgument('use_3d_lidar', default_value='true')

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
    use_3d = LaunchConfiguration('use_3d_lidar')
    xacro_file_default = os.path.join(simulator_dir, 'urdf', 'xacro', 'gazebo', 'mbot_with_fisheye_gazebo.xacro')
    xacro_file_3d = os.path.join(simulator_dir, 'urdf', 'xacro', 'gazebo', 'mbot_with_lidar3d_gazebo.xacro')

    # 先用 3D 版本
    xacro_file = xacro_file_3d
    robot_description_content = xacro.process_file(xacro_file).toxml()

    tmp_urdf = tempfile.NamedTemporaryFile(mode='w', suffix='.urdf', delete=False)
    tmp_urdf.write(robot_description_content)
    tmp_urdf.close()

    spawn_model = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'mrobot',
            '-file', tmp_urdf.name,
            '-timeout', '60'
        ],
        output='screen'
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'robot_description': robot_description_content}]
    )

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

    return LaunchDescription([
        set_gazebo_model_path,
        world_name_arg, use_sim_time_arg, gui_arg, paused_arg, headless_arg, debug_arg,
        use_3d_lidar_arg,
        gazebo,
        spawn_model,
        joint_state_publisher,
        robot_state_publisher,
    ])
