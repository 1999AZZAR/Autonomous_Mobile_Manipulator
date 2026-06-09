#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    pkg_my_robot_bringup = get_package_share_directory('my_robot_bringup')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    enable_mega_communication = LaunchConfiguration('enable_mega_communication', default='true')

    # Robot launch
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_my_robot_bringup, 'launch', 'robot.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )

    # Main application (Flask web UI + REST API + sensor management)
    main_app = ExecuteProcess(
        cmd=['python3', 'src/my_robot_automation/scripts/main.py'],
        output='screen',
        condition=IfCondition(enable_mega_communication)
    )

    # ROS2 interface node (optional, only if ROS2 is available)
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

        robot_launch,
        main_app,
        ros2_interface_node,
    ])
