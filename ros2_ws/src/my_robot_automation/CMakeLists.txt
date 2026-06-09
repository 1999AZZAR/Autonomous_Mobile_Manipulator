cmake_minimum_required(VERSION 3.8)
project(my_robot_automation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate custom interfaces
set(msg_files
  "msg/RobotStatus.msg"
  "msg/AutomationTask.msg"
  "msg/SafetyStatus.msg"
  "msg/SensorData.msg"
  "msg/TaskStatus.msg"
  "msg/NavigationStatus.msg"
)

set(srv_files
  "srv/ExecutePickPlace.srv"
  "srv/ExecutePatrol.srv"
  "srv/ExecuteObstacleAvoidance.srv"
  "srv/GetRobotStatus.srv"
  "srv/EmergencyStop.srv"
  "srv/SetRobotMode.srv"
  "srv/GetSensorData.srv"
  "srv/GetTaskStatus.srv"
  "srv/CancelTask.srv"
  "srv/GetNavigationStatus.srv"
  "srv/ControlGripper.srv"
  "srv/SetGripperTilt.srv"
  "srv/MoveRobot.srv"
  "srv/ControlContainer.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs nav_msgs builtin_interfaces
)

# Install Python executables (only scripts that exist)
install(PROGRAMS
  scripts/main.py
  scripts/app.py
  scripts/config.py
  scripts/mega_interface.py
  scripts/sensor_manager.py
  scripts/gpio_controller.py
  scripts/ros2_interface.py
  scripts/path_planning.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
