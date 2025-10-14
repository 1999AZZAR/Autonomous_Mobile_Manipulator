#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, Command, FindExecutable
from launch_ros.actions import Node

def generate_launch_description():
    # Get package directories
    pkg_my_robot_bringup = get_package_share_directory('my_robot_bringup')
    pkg_my_robot_description = get_package_share_directory('my_robot_description')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Robot description file
    robot_description_file = os.path.join(
        pkg_my_robot_description,
        'urdf',
        'my_robot.urdf.xacro'
    )

    # Robot description content (xacro processed)
    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ',
        robot_description_file,
        ' use_ros2_control:=true',
        ' use_sim_time:=', use_sim_time
    ])

    # Robot state publisher
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

    # Gazebo server
    gazebo_server = ExecuteProcess(
        cmd=[
            'gzserver',
            os.path.join(pkg_my_robot_bringup, 'worlds', 'competition_arena.world'),
            '--verbose'
        ],
        output='screen'
    )

    # Gazebo client (GUI)
    gazebo_client = ExecuteProcess(
        cmd=['gzclient', '--verbose'],
        output='screen',
        environment={'QT_QPA_PLATFORM': 'xcb'}
    )

    # Spawn robot (delayed to allow Gazebo to start)
    spawn_robot = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
            '-entity', 'my_robot',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1'
        ],
        output='screen'
    )

    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        # Robot state publisher
        robot_state_publisher_node,

        # Gazebo server
        gazebo_server,

        # Gazebo client (GUI) - start after server
        TimerAction(
            period=3.0,
            actions=[gazebo_client]
        ),

        # Spawn robot - start after Gazebo
        TimerAction(
            period=5.0,
            actions=[spawn_robot]
        ),
    ])
