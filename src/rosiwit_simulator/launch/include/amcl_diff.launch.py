from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Arguments
    use_map_topic_arg = DeclareLaunchArgument('use_map_topic', default_value='true')
    scan_topic_arg = DeclareLaunchArgument('scan_topic', default_value='scan')
    initial_pose_x_arg = DeclareLaunchArgument('initial_pose_x', default_value='0.0')
    initial_pose_y_arg = DeclareLaunchArgument('initial_pose_y', default_value='0.0')
    initial_pose_a_arg = DeclareLaunchArgument('initial_pose_a', default_value='0.0')
    odom_frame_id_arg = DeclareLaunchArgument('odom_frame_id', default_value='odom')
    base_frame_id_arg = DeclareLaunchArgument('base_frame_id', default_value='base_footprint')
    global_frame_id_arg = DeclareLaunchArgument('global_frame_id', default_value='map')

    # AMCL node (diff drive model)
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[{
            'use_map_topic': LaunchConfiguration('use_map_topic'),
            'odom_model_type': 'diff',
            'odom_alpha5': 0.1,
            'gui_publish_rate': 10.0,
            'laser_max_beams': 60,
            'laser_max_range': 6.0,
            'min_particles': 2000,
            'max_particles': 5000,
            'kld_err': 0.05,
            'kld_z': 0.99,
            'odom_alpha1': 0.2,
            'odom_alpha2': 0.2,
            'odom_alpha3': 0.8,
            'odom_alpha4': 0.2,
            'laser_z_hit': 0.5,
            'laser_z_short': 0.05,
            'laser_z_max': 0.05,
            'laser_z_rand': 0.5,
            'laser_sigma_hit': 0.2,
            'laser_lambda_short': 0.1,
            'laser_model_type': 'likelihood_field',
            'laser_likelihood_max_dist': 2.0,
            'update_min_d': 0.25,
            'update_min_a': 0.2,
            'odom_frame_id': LaunchConfiguration('odom_frame_id'),
            'base_frame_id': LaunchConfiguration('base_frame_id'),
            'global_frame_id': LaunchConfiguration('global_frame_id'),
            'resample_interval': 0.5,
            'transform_tolerance': 1.0,
            'recovery_alpha_slow': 0.0,
            'recovery_alpha_fast': 0.0,
        }],
        remappings=[
            ('scan', LaunchConfiguration('scan_topic')),
        ]
    )

    return LaunchDescription([
        use_map_topic_arg,
        scan_topic_arg,
        initial_pose_x_arg,
        initial_pose_y_arg,
        initial_pose_a_arg,
        odom_frame_id_arg,
        base_frame_id_arg,
        global_frame_id_arg,
        amcl_node,
    ])
