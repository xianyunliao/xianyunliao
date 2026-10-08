import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    # turtlebot3_navigation2
    turtlebot3_navigation2_dir = get_package_share_directory(
        'turtlebot3_navigation2'
    )

    # TurtleBot3 Burger Nav2 parameters
    params_file = os.path.join(
        os.path.expanduser('~/turtlebot3_ws/src/bot2/config'),
        'nav2_params.yaml'
    )

    # Nav2 navigation core only
    # 不启动 AMCL
    # 不启动 Map Server
    # 地图由 slam_gmapping 提供
    navigation_launch = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'navigation_launch.py'
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            navigation_launch
        ),
        launch_arguments={
            'use_sim_time': 'True',
            'params_file': params_file,
            'autostart': 'True'
        }.items()
    )

    return LaunchDescription([
        nav2
    ])
