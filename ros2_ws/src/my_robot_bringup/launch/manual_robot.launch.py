#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # First, process the URDF
    process_urdf = ExecuteProcess(
        cmd=['bash', '-c', 'source /opt/ros/iron/setup.bash && xacro /root/ros2_ws/src/my_robot_description/urdf/my_robot.urdf.xacro > /tmp/robot.urdf'],
        output='screen'
    )
    
    # Then start robot state publisher with the processed URDF
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': 'file:///tmp/robot.urdf',
            'use_sim_time': use_sim_time,
            'publish_frequency': 50.0
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        process_urdf,
        robot_state_publisher_node,
    ])
