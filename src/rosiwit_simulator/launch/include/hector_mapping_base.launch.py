from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Arguments
    tf_map_scanmatch_arg = DeclareLaunchArgument(
        'tf_map_scanmatch_transform_frame_name', default_value='scanmatcher_frame')
    base_frame_arg = DeclareLaunchArgument('base_frame', default_value='base_footprint')
    odom_frame_arg = DeclareLaunchArgument('odom_frame', default_value='odom')
    pub_map_odom_arg = DeclareLaunchArgument('pub_map_odom_transform', default_value='true')
    scan_queue_arg = DeclareLaunchArgument('scan_subscriber_queue_size', default_value='5')
    scan_topic_arg = DeclareLaunchArgument('scan_topic', default_value='scan')
    map_size_arg = DeclareLaunchArgument('map_size', default_value='2048')

    # hector_mapping 节点
    hector_mapping = Node(
        package='hector_mapping',
        executable='hector_mapping',
        name='hector_mapping',
        output='screen',
        parameters=[{
            # Frame names
            'map_frame': 'map',
            'base_frame': LaunchConfiguration('base_frame'),
            'odom_frame': LaunchConfiguration('odom_frame'),
            # Tf use
            'use_tf_scan_transformation': True,
            'use_tf_pose_start_estimate': False,
            'pub_map_odom_transform': LaunchConfiguration('pub_map_odom_transform'),
            # Map size / start point
            'map_resolution': 0.050,
            'map_size': LaunchConfiguration('map_size'),
            'map_start_x': 0.5,
            'map_start_y': 0.5,
            'laser_z_min_value': -1.0,
            'laser_z_max_value': 1.0,
            'map_multi_res_levels': 2,
            # Map update parameters
            'update_factor_free': 0.4,
            'update_factor_occupied': 0.9,
            'map_update_distance_thresh': 0.4,
            'map_update_angle_thresh': 0.06,
            'map_pub_period': 2,
            'laser_min_dist': 0.4,
            'laser_max_dist': 5.5,
            'output_timing': False,
            'pub_map_scanmatch_transform': True,
            # Advertising config
            'advertise_map_service': True,
            'scan_subscriber_queue_size': LaunchConfiguration('scan_subscriber_queue_size'),
            'scan_topic': LaunchConfiguration('scan_topic'),
            # Transform frame
            'tf_map_scanmatch_transform_frame_name': LaunchConfiguration('tf_map_scanmatch_transform_frame_name'),
        }]
    )

    return LaunchDescription([
        tf_map_scanmatch_arg,
        base_frame_arg,
        odom_frame_arg,
        pub_map_odom_arg,
        scan_queue_arg,
        scan_topic_arg,
        map_size_arg,
        hector_mapping,
    ])
