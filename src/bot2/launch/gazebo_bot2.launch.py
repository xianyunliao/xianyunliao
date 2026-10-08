import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    # ============================================================
    # bot2 package
    # ============================================================
    bot2_dir = get_package_share_directory('bot2')

    # ============================================================
    # Gazebo world
    # ============================================================
    world_file = os.path.join(
        bot2_dir,
        'worlds',
        'bot2_room.sdf'
    )

    # ============================================================
    # TurtleBot3 Gazebo models
    # ============================================================
    gazebo_models_dir = os.path.join(
        os.path.expanduser('~/turtlebot3_ws/src'),
        'turtlebot3_simulations',
        'turtlebot3_gazebo',
        'models'
    )

    # ============================================================
    # turtlebot3_gazebo package
    # ============================================================
    turtlebot3_gazebo_dir = get_package_share_directory(
        'turtlebot3_gazebo'
    )

    # ============================================================
    # Robot State Publisher
    # ============================================================
    robot_state_publisher_launch = os.path.join(
        turtlebot3_gazebo_dir,
        'launch',
        'robot_state_publisher.launch.py'
    )

    # ============================================================
    # Gazebo
    # ============================================================
    gazebo = ExecuteProcess(
        cmd=[
            'ign',
            'gazebo',
            '--force-version',
            '6',
            '-r',
            world_file
        ],
        additional_env={
            'IGN_GAZEBO_RESOURCE_PATH': gazebo_models_dir,
            'LIBGL_ALWAYS_SOFTWARE': '1',
            'GALLIUM_DRIVER': 'llvmpipe',
            'QT_QPA_PLATFORM': 'xcb'
        },
        output='screen'
    )

    # ============================================================
    # Robot State Publisher
    # ============================================================
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            robot_state_publisher_launch
        )
    )

    # ============================================================
    # ROS 2 <-> Gazebo Bridge
    #
    # /clock
    # /scan
    # /odom
    # /tf
    # /imu
    # /cmd_vel
    # ============================================================
    bridge = ExecuteProcess(
        cmd=[
            'ros2',
            'run',
            'ros_gz_bridge',
            'parameter_bridge',

            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',

            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',

            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',

            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',

            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',

            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',

            '/world/bot2_complex_room/model/burger/link/camera_link/sensor/rgbd_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',

            '/world/bot2_complex_room/model/burger/link/camera_link/sensor/rgbd_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',

            '/world/bot2_complex_room/model/burger/link/camera_link/sensor/rgbd_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',

            '/world/bot2_complex_room/model/burger/link/camera_link/sensor/rgbd_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'
            
            
        ],
        output='screen'
    )

    # ============================================================
    # Map Server
    # ============================================================
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {
                'yaml_filename':
                    '/home/xianyun/turtlebot3_ws/maps/bot2map.yaml'
            }
        ]
    )

    # ============================================================
    # Map -> Odom static TF
    #
    # 当前实验中 map 与 odom 使用同一坐标系
    # ============================================================
    map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_tf',
        output='screen',
        arguments=[
            '0', '0', '0',
            '0', '0', '0',
            'map',
            'odom'
        ]
    )

    # ============================================================
    # Lifecycle Manager
    #
    # 自动 configure + activate map_server
    # ============================================================
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[
            {
                'autostart': True,
                'node_names': ['map_server'],
                'bond_timeout': 0.0
            }
        ]
    )

    # ============================================================
    # LaunchDescription
    # ============================================================
    return LaunchDescription([

        gazebo,

        robot_state_publisher,

        bridge,

        map_server,

        map_to_odom,

        lifecycle_manager

    ])
