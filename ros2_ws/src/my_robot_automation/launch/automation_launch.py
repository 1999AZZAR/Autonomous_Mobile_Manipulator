#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    # Get package directories
    pkg_my_robot_automation = get_package_share_directory('my_robot_automation')
    pkg_my_robot_bringup = get_package_share_directory('my_robot_bringup')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    enable_rest_api = LaunchConfiguration('enable_rest_api', default='true')
    enable_websocket = LaunchConfiguration('enable_websocket', default='true')
    enable_n8n_bridge = LaunchConfiguration('enable_n8n_bridge', default='true')
    
    # Robot launch
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_my_robot_bringup, 'launch', 'robot.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
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

    # REST API server
    rest_api_server = Node(
        package='my_robot_automation',
        executable='rest_api_server.py',
        name='rest_api_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        condition=IfCondition(enable_rest_api)
    )

    # WebSocket server
    websocket_server = Node(
        package='my_robot_automation',
        executable='websocket_server.py',
        name='websocket_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        condition=IfCondition(enable_websocket)
    )

    # n8n-ROS2 bridge
    n8n_bridge = Node(
        package='my_robot_automation',
        executable='n8n_ros2_bridge.py',
        name='n8n_ros2_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        condition=IfCondition(enable_n8n_bridge)
    )

    # Web robot interface
    web_robot_interface = Node(
        package='my_robot_automation',
        executable='web_robot_interface.py',
        name='web_robot_interface',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        condition=IfCondition(enable_rest_api)
    )
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('enable_rest_api', default_value='true'),
        DeclareLaunchArgument('enable_websocket', default_value='true'),
        DeclareLaunchArgument('enable_n8n_bridge', default_value='true'),
        
        # Launch robot system
        robot_launch,
        
        # Launch automation services
        automation_server,
        rest_api_server,
        websocket_server,
        n8n_bridge,
        web_robot_interface,
    ])
