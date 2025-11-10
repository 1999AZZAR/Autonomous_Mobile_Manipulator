#!/usr/bin/env python3

"""
Launch file for TF-Luna single-point LIDAR sensor
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Launch arguments
    serial_port = LaunchConfiguration('serial_port', default='/dev/serial0')
    baud_rate = LaunchConfiguration('baud_rate', default='115200')
    frame_id = LaunchConfiguration('frame_id', default='tf_luna')
    field_of_view = LaunchConfiguration('field_of_view', default='0.087')  # ~5 degrees
    min_range = LaunchConfiguration('min_range', default='0.1')
    max_range = LaunchConfiguration('max_range', default='8.0')
    publish_rate = LaunchConfiguration('publish_rate', default='10.0')

    # TF-Luna sensor node
    tf_luna_sensor = Node(
        package='my_robot_automation',
        executable='tf_luna_sensor.py',
        name='tf_luna_sensor',
        output='screen',
        parameters=[{
            'serial_port': serial_port,
            'baud_rate': baud_rate,
            'frame_id': frame_id,
            'field_of_view': field_of_view,
            'min_range': min_range,
            'max_range': max_range,
            'publish_rate': publish_rate
        }],
        remappings=[
            ('tf_luna/range', 'tf_luna/range')  # Explicit topic mapping
        ]
    )

    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument('serial_port', default_value='/dev/serial0',
                            description='Serial port for TF-Luna sensor'),
        DeclareLaunchArgument('baud_rate', default_value='115200',
                            description='Baud rate for serial communication'),
        DeclareLaunchArgument('frame_id', default_value='tf_luna',
                            description='ROS TF frame ID for the sensor'),
        DeclareLaunchArgument('field_of_view', default_value='0.087',
                            description='Field of view in radians (~5 degrees)'),
        DeclareLaunchArgument('min_range', default_value='0.1',
                            description='Minimum detection range in meters'),
        DeclareLaunchArgument('max_range', default_value='8.0',
                            description='Maximum detection range in meters'),
        DeclareLaunchArgument('publish_rate', default_value='10.0',
                            description='Publishing rate in Hz'),

        # Launch the sensor node
        tf_luna_sensor
    ])
