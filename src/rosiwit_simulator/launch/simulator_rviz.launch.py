import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro


def generate_launch_description():
    simulator_dir = get_package_share_directory('rosiwit_simulator')

    # Model argument
    model_arg = DeclareLaunchArgument(
        'model',
        default_value=os.path.join(simulator_dir, 'urdf', 'xacro', 'gazebo', 'mbot_with_laser_gazebo.xacro')
    )

    # GUI argument
    gui_arg = DeclareLaunchArgument('gui', default_value='false')

    # Process xacro file
    xacro_file = os.path.join(simulator_dir, 'urdf', 'xacro', 'gazebo', 'mbot_with_laser_gazebo.xacro')
    robot_description_content = xacro.process_file(xacro_file).toxml()

    # RViz2
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', os.path.join(simulator_dir, 'rviz', 'urdf.rviz')],
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
        }]
    )

    return LaunchDescription([
        model_arg,
        gui_arg,
        rviz,
    ])
