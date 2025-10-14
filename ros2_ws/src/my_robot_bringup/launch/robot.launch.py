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

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_ros2_control = LaunchConfiguration('use_ros2_control', default='true')

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
        ' use_ros2_control:=', use_ros2_control,
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

    # Joint state publisher GUI
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
    )

    # Controller manager
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            os.path.join(pkg_my_robot_description, 'config', 'controllers.yaml'),
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    # Spawn controllers
    spawn_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )

    spawn_omni_wheels_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'omni_wheels_controller'],
        output='screen'
    )

    spawn_lifter_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'lifter_controller'],
        output='screen'
    )

    spawn_servo_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'servo_controller'],
        output='screen'
    )

    # IMU broadcaster
    imu_sensor_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['imu_sensor_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # Laser scan matcher for odometry
    laser_scan_matcher_node = Node(
        package='laser_scan_matcher',
        executable='laser_scan_matcher_node',
        name='laser_scan_matcher_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'base_frame': 'base_link',
            'odom_frame': 'odom',
            'fixed_frame': 'odom'
        }]
    )

    # Robot localization (EKF fusion)
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(pkg_my_robot_bringup, 'config', 'ekf.yaml')]
    )

    # LIDAR
    lidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='lidar_composition',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 115200,
            'frame_id': 'lidar_frame',
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'Standard'
        }],
        output='screen'
    )

    # IMU filter
    imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_madgwick_node',
        output='screen',
        parameters=[{
            'use_mag': False,
            'publish_tf': False,
            'world_frame': 'enu'
        }]
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
            'certfile': '',
            'keyfile': '',
            'topic_whitelist': ['.*'],
            'service_whitelist': ['.*'],
            'param_whitelist': ['.*'],
            'send_buffer_limit': 1000000,
            'use_compression': False,
            'calibration_file': ''
        }]
    )

    # Joystick teleop
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'device_id': 0,
            'deadzone': 0.05,
            'autorepeat_rate': 20.0
        }]
    )

    # Teleop twist joy for joystick control
    teleop_twist_joy_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[os.path.join(pkg_my_robot_bringup, 'config', 'teleop_joy.yaml')],
        remappings=[
            ('cmd_vel', '/omni_wheels_controller/cmd_vel_unstamped')
        ]
    )

    # Keyboard teleop
    teleop_twist_keyboard_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard_node',
        output='screen',
        prefix='xterm -e',
        remappings=[
            ('cmd_vel', '/omni_wheels_controller/cmd_vel_unstamped')
        ]
    )

    # Static transform publishers for sensors
    base_to_laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf',
        arguments=['0', '0', '0.2', '0', '0', '0', 'base_link', 'lidar_frame']
    )

    base_to_imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_imu_tf',
        arguments=['0.1', '0', '0.1', '0', '0', '0', 'base_link', 'imu_link']
    )

    base_to_camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_tf',
        arguments=['0.2', '0', '0.3', '0', '0', '0', 'base_link', 'camera_link']
    )

    # Return launch description
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_ros2_control', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),

        # Core robot nodes
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        controller_manager_node,

        # Controller spawning
        RegisterEventHandler(
            OnProcessExit(
                target_action=controller_manager_node,
                on_exit=[
                    spawn_joint_state_broadcaster,
                    spawn_omni_wheels_controller,
                    spawn_lifter_controller,
                    spawn_servo_controller,
                ]
            )
        ),

        # Sensor nodes
        rplidar_node,
        imu_filter_node,

        # Localization and odometry
        robot_localization_node,
        laser_scan_matcher_node,

        # Static transforms
        base_to_laser_tf,
        base_to_imu_tf,
        base_to_camera_tf,

        # Teleoperation
        joy_node,
        teleop_twist_joy_node,
        teleop_twist_keyboard_node,

        # Visualization and monitoring
        foxglove_bridge_node,
    ])
