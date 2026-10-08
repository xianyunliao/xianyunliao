from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        # base_link -> camera_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_link_tf',
            arguments=[
                '0.12', '0', '0.20',
                '0', '0', '0',
                'base_link',
                'camera_link'
            ],
            output='screen'
        ),

        # camera_link -> camera_optical_frame
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_optical_tf',
            arguments=[
                '0', '0', '0',
                '-1.5708', '0', '-1.5708',
                'camera_link',
                'camera_optical_frame'
            ],
            output='screen'
        ),

        # RGB-D color detector
        Node(
            package='vision_detector',
            executable='color_detector',
            name='color_detector',
            output='screen'
        ),
    ])
