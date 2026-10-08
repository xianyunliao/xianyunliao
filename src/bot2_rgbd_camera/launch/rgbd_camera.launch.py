import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    package_dir = get_package_share_directory('bot2_rgbd_camera')

    rgb_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # ----------------------------------------------------
            # RGB Image
            # ----------------------------------------------------

            '/world/rgbd_test_world/model/turtlebot3_burger_rgbd/link/base_footprint/sensor/rgbd_camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',

            # ----------------------------------------------------
            # Depth Image
            # ----------------------------------------------------

            '/world/rgbd_test_world/model/turtlebot3_burger_rgbd/link/base_footprint/sensor/rgbd_camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',

            # ----------------------------------------------------
            # Camera Info
            # ----------------------------------------------------

            '/world/rgbd_test_world/model/turtlebot3_burger_rgbd/link/base_footprint/sensor/rgbd_camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',

            # ----------------------------------------------------
            # Point Cloud
            # ----------------------------------------------------

            '/world/rgbd_test_world/model/turtlebot3_burger_rgbd/link/base_footprint/sensor/rgbd_camera/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked'
        ],
        output='screen'
    )

    world_file = os.path.join(
        package_dir,
        'worlds',
        'rgbd_test.world'
    )

    robot_urdf = os.path.join(
        package_dir,
        'urdf',
        'turtlebot3_burger_rgbd_converted.urdf'
    )

    with open(robot_urdf, 'r') as file:
        robot_description = file.read()

    # Gazebo resource path
    turtlebot_description_dir = os.path.expanduser(
        '~/turtlebot3_ws/src/turtlebot3/turtlebot3_description'
    )


    turtlebot_src_dir = os.path.expanduser(
        '~/turtlebot3_ws/src/turtlebot3'
    )

    set_gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=f'{turtlebot_src_dir}:{turtlebot_description_dir}'
    )

    # Gazebo 6 / Ignition Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': f'-r -v 4 "{world_file}"'
        }.items()
    )

    # Publish robot TF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': robot_description,
                'use_sim_time': True
            }
        ]
    )

    # Spawn robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'turtlebot3_burger_rgbd',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.15'
        ],
        output='screen'
    )

    return LaunchDescription([
        set_gazebo_resource_path,
        gazebo,
        robot_state_publisher,
        spawn_robot,
        rgb_bridge
    ])
