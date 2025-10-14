#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Get package directories
    pkg_my_robot_bringup = get_package_share_directory('my_robot_bringup')
    pkg_my_robot_description = get_package_share_directory('my_robot_description')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file = LaunchConfiguration('world', default='competition_arena.world')

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

    # Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': os.path.join(pkg_my_robot_bringup, 'worlds', world_file),
            'use_sim_time': use_sim_time,
        }.items()
    )

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

    # Spawn robot in Gazebo
    spawn_robot_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_robot',
        arguments=[
            '-entity', 'my_robot',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1'
        ],
        output='screen'
    )

    # Joint state publisher GUI for simulation
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # Controller manager for simulation
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            os.path.join(pkg_my_robot_description, 'config', 'controllers.yaml'),
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    # Spawn controllers for simulation
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

    # IMU broadcaster for simulation
    imu_sensor_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['imu_sensor_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # Static transform publishers for simulation
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


    # Load joint state broadcaster and controllers
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )

    load_omni_wheels_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'omni_wheels_controller'],
        output='screen'
    )

    # Return launch description
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value='competition_arena.world'),

        # Gazebo simulation
        gazebo_launch,

        # Robot description and spawning
        robot_state_publisher_node,
        spawn_robot_node,

        # Joint state and GUI
        joint_state_publisher_gui_node,

        # Controllers
        controller_manager_node,

        # Load controllers after manager starts
        RegisterEventHandler(
            OnProcessExit(
                target_action=controller_manager_node,
                on_exit=[
                    load_joint_state_broadcaster,
                    load_omni_wheels_controller,
                ]
            )
        ),

        # Static transforms
        base_to_laser_tf,
        base_to_imu_tf,
        base_to_camera_tf,

        # IMU broadcaster
        RegisterEventHandler(
            OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[imu_sensor_broadcaster]
            )
        ),
    ])
