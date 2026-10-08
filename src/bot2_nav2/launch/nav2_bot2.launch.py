#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')

    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true'
        ),

        DeclareLaunchArgument(
            'params_file',
            default_value=(
                '/home/xianyun/turtlebot3_ws/src/'
                'bot2_nav2/config/nav2_params.yaml'
            )
        ),

        # ==================================================
        # MAP SERVER
        # ==================================================

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                params_file,
                {
                    'use_sim_time': use_sim_time,
                    'yaml_filename':
    '/home/xianyun/turtlebot3_ws/maps/bot2map.yaml'
                }
            ]
        ),

        # ==================================================
        # AMCL
        # ==================================================

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[
                params_file,
                {
                    'use_sim_time': use_sim_time
                }
            ]
        ),

        # ==================================================
        # PLANNER SERVER
        # ==================================================

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[
                params_file,
                {
                    'use_sim_time': use_sim_time
                }
            ]
        ),

        # ==================================================
        # CONTROLLER SERVER
        # ==================================================

        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[
                params_file,
                {
                    'use_sim_time': use_sim_time
                }
            ]
        ),

        # ==================================================
        # BT NAVIGATOR
        # ==================================================

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[
                params_file,
                {
                    'use_sim_time': use_sim_time
                }
            ]
        ),

        # ==================================================
        # BEHAVIOR SERVER
        # ==================================================

        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[
                params_file,
                {
                    'use_sim_time': use_sim_time
                }
            ]
        ),

        # ==================================================
        # LIFECYCLE MANAGER
        # ==================================================

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[
                params_file,
                {
                    'use_sim_time': use_sim_time,
                    'autostart': True,

                    'node_names': [
                        'map_server',
                        'amcl',
                        'planner_server',
                        'controller_server',
                        'bt_navigator',
                        'behavior_server'
                    ]
                }
            ]
        ),
    ])
