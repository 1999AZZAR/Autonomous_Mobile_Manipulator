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
find_package(control_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate custom interfaces
set(msg_files
  "msg/RobotStatus.msg"
  "msg/AutomationTask.msg"
  "msg/SafetyStatus.msg"
)

set(srv_files
  "srv/ExecutePickPlace.srv"
  "srv/ExecutePatrol.srv"
  "srv/ExecuteObstacleAvoidance.srv"
  "srv/GetRobotStatus.srv"
  "srv/EmergencyStop.srv"
  "srv/SetRobotMode.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs nav_msgs
)

# Install Python executables
install(PROGRAMS
  scripts/robot_automation_server.py
  scripts/pick_place_server.py
  scripts/patrol_server.py
  scripts/obstacle_avoidance_server.py
  scripts/emergency_stop_server.py
  scripts/robot_status_server.py
  scripts/rest_api_server.py
  scripts/websocket_server.py
  scripts/mqtt_bridge.py
  scripts/n8n_ros2_bridge.py
  scripts/dummy_sensor_actuator.py
  scripts/robot_controller.py
  scripts/web_robot_interface.py
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
