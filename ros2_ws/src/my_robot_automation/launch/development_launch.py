#!/usr/bin/env python3
"""
Development Launch File for Autonomous Mobile Manipulator
Launches the robot control system in simulation mode for development/testing
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Main application in simulation mode
    main_app = ExecuteProcess(
        cmd=['python3', 'src/my_robot_automation/scripts/main.py', '--simulation'],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        main_app,
    ])
