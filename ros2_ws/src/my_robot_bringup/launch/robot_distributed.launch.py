#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import launch_ros.actions
import launch.conditions

def generate_launch_description():
    # Get package directories
    pkg_my_robot_bringup = get_package_share_directory('my_robot_bringup')
    pkg_my_robot_description = get_package_share_directory('my_robot_description')
    pkg_my_robot_automation = get_package_share_directory('my_robot_automation')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    mega_port = LaunchConfiguration('mega_port', default='/dev/ttyACM0')
    mega_baudrate = LaunchConfiguration('mega_baudrate', default='115200')

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
        ' use_ros2_control:=false',  # No ROS2 control in distributed mode
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
            'use_sim_time': use_sim_time
        }]
    )

    # Mega Serial Interface - handles communication with Arduino Mega
    mega_serial_interface_node = Node(
        package='my_robot_automation',
        executable='mega_serial_interface',
        name='mega_serial_interface',
        output='screen',
        parameters=[{
            'port': mega_port,
            'baudrate': mega_baudrate
        }]
    )

    # Real Robot Sensor Actuator - handles RPi sensors (IMU, LiDAR, Camera)
    real_robot_sensor_actuator_node = Node(
        package='my_robot_automation',
        executable='real_robot_sensor_actuator_updated',
        name='real_robot_sensor_actuator',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

    # TF-Luna LiDAR node
    lidar_node = Node(
        package='tf_luna_sensor',
        executable='tf_luna_sensor_node',
        name='tf_luna_sensor',
        output='screen',
        parameters=[{
            'port': '/dev/ttyUSB0',  # Adjust as needed
            'frame_id': 'tf_luna'
        }]
    )

    # Camera node (USB camera)
    camera_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        output='screen',
        parameters=[{
            'video_device': '/dev/video0',  # Adjust as needed
            'image_width': 640,
            'image_height': 480,
            'pixel_format': 'mjpeg',
            'camera_frame_id': 'camera_link',
            'io_method': 'mmap'
        }]
    )

    # IMU node (MPU6050)
    imu_node = Node(
        package='my_robot_automation',
        executable='mpu6050_reader',
        name='mpu6050_reader',
        output='screen',
        parameters=[{
            'frame_id': 'imu_link',
            'i2c_address': 0x68,
            'i2c_bus': 1
        }]
    )

    # Robot Controller (high-level control)
    robot_controller_node = Node(
        package='my_robot_automation',
        executable='robot_controller',
        name='robot_controller',
        output='screen'
    )

    # Foxglove bridge for visualization
    foxglove_bridge_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{
            'port': 8765,
            'address': '0.0.0.0',
            'tls': False,
            'capabilities': ['clientPublish', 'parametersSubscribe', 'services'],
            'num_threads': 0
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
                            description='Use simulation time'),
        DeclareLaunchArgument('mega_port', default_value='/dev/ttyACM0',
                            description='Arduino Mega serial port'),
        DeclareLaunchArgument('mega_baudrate', default_value='115200',
                            description='Arduino Mega baudrate'),

        # Core nodes
        robot_state_publisher_node,
        mega_serial_interface_node,
        real_robot_sensor_actuator_node,

        # Sensor nodes (RPi side)
        lidar_node,
        camera_node,
        imu_node,

        # Control nodes
        robot_controller_node,

        # Visualization
        foxglove_bridge_node
    ])
