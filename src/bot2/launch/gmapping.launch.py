from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    gmapping = Node(
        package='slam_gmapping',
        executable='slam_gmapping',
        name='slam_gmapping',
        output='screen',
        parameters=[
            {
                'use_sim_time': True,
                'transform_publish_period': 0.05,
                'base_frame': 'base_footprint',
                'odom_frame': 'odom',
                'map_frame': 'map',
            }
        ]
    )

    return LaunchDescription([
        gmapping
    ])
