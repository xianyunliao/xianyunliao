import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


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

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            robot_state_publisher_launch
        )
    )

    # ============================================================
    # Gazebo
    #
    # 继续使用已经验证稳定的 Gazebo Sim 6.18
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
    # ROS 2 <-> Gazebo Bridge
    #
    # SLAM 必需：
    #   /clock
    #   /scan
    #   /odom
    #   /tf
    #
    # 导航必需：
    #   /cmd_vel
    #
    # RGB-D 保留，后面还可以继续使用视觉功能
    # ============================================================
    bridge = ExecuteProcess(
        cmd=[
            'ros2',
            'run',
            'ros_gz_bridge',
            'parameter_bridge',

            # simulation time
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',

            # LaserScan
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',

            # Odometry
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',

            # TF
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',

            # IMU
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',

            # Velocity command
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',

            # ====================================================
            # RGB-D camera
            # ====================================================
            '/world/bot2_complex_room/model/burger/link/camera_link/sensor/rgbd_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',

            '/world/bot2_complex_room/model/burger/link/camera_link/sensor/rgbd_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',

            '/world/bot2_complex_room/model/burger/link/camera_link/sensor/rgbd_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',

            '/world/bot2_complex_room/model/burger/link/camera_link/sensor/rgbd_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'
        ],
        output='screen'
    )

    # ============================================================
    # 注意：
    #
    # 这里故意没有：
    #
    # map_server
    # map_to_odom
    # lifecycle_manager
    #
    # 因为这些会和 slam_gmapping 冲突。
    #
    # 后面由 slam_gmapping 发布：
    #
    #       map -> odom
    #
    # ============================================================

    return LaunchDescription([

        gazebo,

        robot_state_publisher,

        bridge

    ])
