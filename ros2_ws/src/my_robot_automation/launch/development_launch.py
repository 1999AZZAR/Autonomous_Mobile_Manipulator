#!/usr/bin/env python3
"""
Development Launch File for LKS Robot
Launches ROS2 automation services with sensor simulation for development/testing
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    enable_rest_api = LaunchConfiguration('enable_rest_api', default='true')
    enable_websocket = LaunchConfiguration('enable_websocket', default='true')
    enable_n8n_bridge = LaunchConfiguration('enable_n8n_bridge', default='true')

    # Sensor Simulator (replaces real hardware for development)
    sensor_simulator = Node(
        package='my_robot_automation',
        executable='sensor_simulator.py',
        name='sensor_simulator',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

    # Automation server
    automation_server = Node(
        package='my_robot_automation',
        executable='robot_automation_server.py',
        name='robot_automation_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

    # Individual service servers
    pick_place_server = Node(
        package='my_robot_automation',
        executable='pick_place_server.py',
        name='pick_place_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

    patrol_server = Node(
        package='my_robot_automation',
        executable='patrol_server.py',
        name='patrol_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

    obstacle_avoidance_server = Node(
        package='my_robot_automation',
        executable='obstacle_avoidance_server.py',
        name='obstacle_avoidance_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

    emergency_stop_server = Node(
        package='my_robot_automation',
        executable='emergency_stop_server.py',
        name='emergency_stop_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

    robot_status_server = Node(
        package='my_robot_automation',
        executable='robot_status_server.py',
        name='robot_status_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

    # New sensor and management servers
    sensor_data_server = Node(
        package='my_robot_automation',
        executable='sensor_data_server.py',
        name='sensor_data_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

    task_management_server = Node(
        package='my_robot_automation',
        executable='task_management_server.py',
        name='task_management_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

    navigation_status_server = Node(
        package='my_robot_automation',
        executable='navigation_status_server.py',
        name='navigation_status_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

    # REST API server
    rest_api_server = Node(
        package='my_robot_automation',
        executable='rest_api_server.py',
        name='rest_api_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

    # WebSocket server
    websocket_server = Node(
        package='my_robot_automation',
        executable='websocket_server.py',
        name='websocket_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

    # n8n-ROS2 bridge
    n8n_bridge = Node(
        package='my_robot_automation',
        executable='n8n_ros2_bridge.py',
        name='n8n_ros2_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

    # Professional web interface (frontend for REST API)
    web_robot_interface = Node(
        package='my_robot_automation',
        executable='web_robot_interface.py',
        name='web_robot_interface',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('enable_rest_api', default_value='true'),
        DeclareLaunchArgument('enable_websocket', default_value='true'),
        DeclareLaunchArgument('enable_n8n_bridge', default_value='true'),

        # Sensor simulation (replaces real hardware)
        sensor_simulator,

        # Launch automation services
        automation_server,

        # Launch individual service servers
        pick_place_server,
        patrol_server,
        obstacle_avoidance_server,
        emergency_stop_server,
        robot_status_server,
        sensor_data_server,
        task_management_server,
        navigation_status_server,

        # Launch API and communication services
        rest_api_server,
        websocket_server,
        n8n_bridge,
        web_robot_interface,
    ])
