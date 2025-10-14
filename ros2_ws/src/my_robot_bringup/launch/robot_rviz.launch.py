#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, FindExecutable
from launch_ros.actions import Node

def generate_launch_description():
    pkg_my_robot_bringup = get_package_share_directory('my_robot_bringup')
    pkg_my_robot_description = get_package_share_directory('my_robot_description')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    robot_description_file = os.path.join(
        pkg_my_robot_description,
        'urdf',
        'my_robot.urdf.xacro'
    )

    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ',
        robot_description_file,
        ' use_ros2_control:=true',
        ' use_sim_time:=', use_sim_time
    ])

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time,
            'publish_frequency': 50.0
        }]
    )

    rviz_config_file = os.path.join(pkg_my_robot_bringup, 'rviz', 'robot_view.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        robot_state_publisher_node,
        rviz_node,
    ])