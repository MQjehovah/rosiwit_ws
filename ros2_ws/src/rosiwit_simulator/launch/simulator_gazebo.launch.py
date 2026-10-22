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

    # WSLg exposes D3D12 as the GPU driver; on native Linux Mesa picks the
    # right one by itself.
    if os.path.exists('/dev/dxg'):
        os.environ.setdefault('LIBGL_ALWAYS_SOFTWARE', '0')
        os.environ.setdefault('GALLIUM_DRIVER', 'd3d12')

    # ROS 2's setup.bash does not source Gazebo's own setup.sh, so these are
    # normally unset. gazebo_ros launch files then override
    # GAZEBO_RESOURCE_PATH with ROS package shares only, Gazebo cannot find
    # media/materials/scripts/gazebo.material, RenderEngine fails to create
    # the scene and gzclient aborts in GLWidget::showEvent.
    gazebo_share = '/usr/share/gazebo-11'
    gazebo_plugins = '/usr/lib/x86_64-linux-gnu/gazebo-11/plugins'

    def prepend_env(name, value):
        if not os.path.isdir(value):
            return
        current = os.environ.get(name, '')
        if value not in current.split(os.pathsep):
            os.environ[name] = os.pathsep.join(filter(None, [value, current]))

    prepend_env('GAZEBO_RESOURCE_PATH', gazebo_share)
    prepend_env('GAZEBO_PLUGIN_PATH', gazebo_plugins)
    prepend_env('GAZEBO_MODEL_PATH', os.path.join(gazebo_share, 'models'))

    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=os.pathsep.join(filter(None, [
            os.path.join(simulator_dir, 'models'),
            os.environ.get('GAZEBO_MODEL_PATH', ''),
        ]))
    )
    set_gazebo_resource_path = SetEnvironmentVariable(
        name='GAZEBO_RESOURCE_PATH',
        value=os.environ.get('GAZEBO_RESOURCE_PATH', '')
    )
    set_gazebo_plugin_path = SetEnvironmentVariable(
        name='GAZEBO_PLUGIN_PATH',
        value=os.environ.get('GAZEBO_PLUGIN_PATH', '')
    )

    world_name_arg = DeclareLaunchArgument(
        'world_name',
        default_value=os.path.join(simulator_dir, 'world', 'house.world')
    )
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    gui_arg = DeclareLaunchArgument('gui', default_value='true')
    paused_arg = DeclareLaunchArgument('paused', default_value='false')
    headless_arg = DeclareLaunchArgument('headless', default_value='false')
    debug_arg = DeclareLaunchArgument('debug', default_value='false')
    use_3d_lidar_arg = DeclareLaunchArgument('use_3d_lidar', default_value='true')
    spawn_x_arg = DeclareLaunchArgument('spawn_x', default_value='0')
    spawn_y_arg = DeclareLaunchArgument('spawn_y', default_value='0')
    spawn_z_arg = DeclareLaunchArgument('spawn_z', default_value='0.1')
    spawn_yaw_arg = DeclareLaunchArgument('spawn_yaw', default_value='0')

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
            '-x', LaunchConfiguration('spawn_x'),
            '-y', LaunchConfiguration('spawn_y'),
            '-z', LaunchConfiguration('spawn_z'),
            '-Y', LaunchConfiguration('spawn_yaw'),
            '-timeout', '60'
        ],
        output='screen'
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': True,
        }]
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'publish_frequency': 50.0,
            'use_sim_time': True,
        }]
    )

    return LaunchDescription([
        set_gazebo_model_path,
        set_gazebo_resource_path,
        set_gazebo_plugin_path,
        world_name_arg, use_sim_time_arg, gui_arg, paused_arg, headless_arg, debug_arg,
        use_3d_lidar_arg, spawn_x_arg, spawn_y_arg, spawn_z_arg, spawn_yaw_arg,
        gazebo,
        spawn_model,
        joint_state_publisher,
        robot_state_publisher,
    ])
