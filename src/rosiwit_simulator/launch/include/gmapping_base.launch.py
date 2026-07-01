from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Arguments
    scan_topic_arg = DeclareLaunchArgument('scan_topic', default_value='scan')
    base_frame_arg = DeclareLaunchArgument('base_frame', default_value='base_footprint')
    odom_frame_arg = DeclareLaunchArgument('odom_frame', default_value='odom')

    # slam_gmapping 节点
    slam_gmapping = Node(
        package='slam_gmapping',
        executable='slam_gmapping',
        name='slam_gmapping',
        output='screen',
        respawn=True,
        parameters=[{
            'base_frame': LaunchConfiguration('base_frame'),
            'odom_frame': LaunchConfiguration('odom_frame'),
            'map_update_interval': 1.0,
            'maxUrange': 8.0,
            'maxRange': 12.0,
            'sigma': 0.05,
            'kernelSize': 1,
            'lstep': 0.05,
            'astep': 0.05,
            'iterations': 5,
            'lsigma': 0.075,
            'ogain': 3.0,
            'lskip': 0,
            'minimumScore': 100,
            'srr': 0.01,
            'srt': 0.02,
            'str': 0.01,
            'stt': 0.02,
            'linearUpdate': 0.05,
            'angularUpdate': 0.0436,
            'temporalUpdate': -1.0,
            'resampleThreshold': 0.5,
            'particles': 8,
            'xmin': -50.0,
            'ymin': -50.0,
            'xmax': 50.0,
            'ymax': 50.0,
            'delta': 0.05,
            'llsamplerange': 0.01,
            'llsamplestep': 0.01,
            'lasamplerange': 0.005,
            'lasamplestep': 0.005,
        }],
        remappings=[
            ('scan', LaunchConfiguration('scan_topic')),
        ]
    )

    return LaunchDescription([
        scan_topic_arg,
        base_frame_arg,
        odom_frame_arg,
        slam_gmapping,
    ])
