#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    enable_mega_communication = LaunchConfiguration('enable_mega_communication', default='true')

    main_app = ExecuteProcess(
        cmd=['python3', 'src/my_robot_automation/scripts/main.py'],
        output='screen',
        condition=IfCondition(enable_mega_communication)
    )

    ros2_interface_node = Node(
        package='my_robot_automation',
        executable='ros2_interface.py',
        name='ros2_interface',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        condition=IfCondition(enable_mega_communication)
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('enable_mega_communication', default_value='true'),
        main_app,
        ros2_interface_node,
    ])
