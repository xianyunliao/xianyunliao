from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',

        name='slam_toolbox',

        output='screen',

        parameters=[{

            # ==================================================
            # ROS 2 / Gazebo
            # ==================================================

            'use_sim_time': True,

            # ==================================================
            # Frames
            # ==================================================

            'map_frame': 'map',

            'odom_frame': 'odom',

            'base_frame': 'base_footprint',

            # ==================================================
            # LaserScan
            # ==================================================

            'scan_topic': '/scan',

            # 异步模式必须为 1
            'scan_queue_size': 1,

            'throttle_scans': 1,

            # ==================================================
            # Mapping
            # ==================================================

            'mode': 'mapping',

            'resolution': 0.05,

            'map_update_interval': 2.0,

            # ==================================================
            # Laser range
            # ==================================================

            'min_laser_range': 0.12,

            'max_laser_range': 6.0,

            # ==================================================
            # TF
            # ==================================================

            'transform_publish_period': 0.02,

            'transform_timeout': 0.2,

            'tf_buffer_duration': 30.0,

            # ==================================================
            # Scan processing
            # ==================================================

            'minimum_time_interval': 0.2,

            'minimum_travel_distance': 0.20,

            'minimum_travel_heading': 0.15,

            'check_min_dist_and_heading_precisely': False,

            # ==================================================
            # Scan matching
            # ==================================================

            'use_scan_matching': True,

            'use_scan_barycenter': True,

            'scan_buffer_size': 10,

            'scan_buffer_maximum_scan_distance': 6.0,

            'link_match_minimum_response_fine': 0.1,

            'link_scan_maximum_distance': 1.5,

            # ==================================================
            # Loop closure
            # ==================================================

            'do_loop_closing': True,

            'loop_search_maximum_distance': 3.0,

            'loop_match_minimum_chain_size': 10,

            'loop_match_maximum_variance_coarse': 3.0,

            'loop_match_minimum_response_coarse': 0.35,

            'loop_match_minimum_response_fine': 0.45,

            # ==================================================
            # Correlation
            # ==================================================

            'correlation_search_space_dimension': 0.5,

            'correlation_search_space_resolution': 0.01,

            'correlation_search_space_smear_deviation': 0.1,

            # ==================================================
            # Loop search
            # ==================================================

            'loop_search_space_dimension': 8.0,

            'loop_search_space_resolution': 0.05,

            'loop_search_space_smear_deviation': 0.03,

            # ==================================================
            # Scan matcher
            # ==================================================

            'distance_variance_penalty': 0.5,

            'angle_variance_penalty': 1.0,

            'fine_search_angle_offset': 0.00349,

            'coarse_search_angle_offset': 0.349,

            'coarse_angle_resolution': 0.0349,

            'minimum_angle_penalty': 0.9,

            'minimum_distance_penalty': 0.5,

            'use_response_expansion': True,

            'min_pass_through': 2,

            'occupancy_threshold': 0.1,

            # ==================================================
            # Ceres optimizer
            # ==================================================

            'solver_plugin':
                'solver_plugins::CeresSolver',

            'ceres_linear_solver':
                'SPARSE_NORMAL_CHOLESKY',

            'ceres_preconditioner':
                'SCHUR_JACOBI',

            'ceres_trust_strategy':
                'LEVENBERG_MARQUARDT',

            'ceres_dogleg_type':
                'TRADITIONAL_DOGLEG',

            # 官方默认先用 None
            'ceres_loss_function': 'None',

            # ==================================================
            # System
            # ==================================================

            'debug_logging': False,

            'enable_interactive_mode': False,

            'stack_size_to_use': 40000000,

        }],

        remappings=[
            ('scan', '/scan'),
        ],
    )

    return LaunchDescription([
        slam_toolbox
    ])
