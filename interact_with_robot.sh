#!/bin/bash

# Script to interact with the robot simulation
# This provides commands to control and monitor the robot

echo "=== Robot Simulation Interaction Script ==="
echo ""

# Check if container is running
if ! docker ps | grep -q ros2_sim_container; then
    echo "Error: ros2_sim_container is not running."
    echo "Please start it first with: docker compose up -d ros2-sim"
    exit 1
fi

echo "Robot simulation is running! Here are some commands you can use:"
echo ""

while true; do
    echo "Choose an option:"
    echo "1. Check robot status"
    echo "2. Move robot forward"
    echo "3. Turn robot left"
    echo "4. Turn robot right"
    echo "5. Stop robot"
    echo "6. View robot topics"
    echo "7. Launch RViz (3D visualization)"
    echo "8. Exit"
    echo ""
    read -p "Enter your choice (1-8): " choice

    case $choice in
        1)
            echo "Checking robot status..."
            docker exec ros2_sim_container bash -c "source /opt/ros/iron/setup.bash && cd /root/ros2_ws && source install/setup.bash && ros2 service call /get_model_list gazebo_msgs/srv/GetModelList"
            ;;
        2)
            echo "Moving robot forward..."
            docker exec ros2_sim_container bash -c "source /opt/ros/iron/setup.bash && cd /root/ros2_ws && source install/setup.bash && ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'"
            ;;
        3)
            echo "Turning robot left..."
            docker exec ros2_sim_container bash -c "source /opt/ros/iron/setup.bash && cd /root/ros2_ws && source install/setup.bash && ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}'"
            ;;
        4)
            echo "Turning robot right..."
            docker exec ros2_sim_container bash -c "source /opt/ros/iron/setup.bash && cd /root/ros2_ws && source install/setup.bash && ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.5}}'"
            ;;
        5)
            echo "Stopping robot..."
            docker exec ros2_sim_container bash -c "source /opt/ros/iron/setup.bash && cd /root/ros2_ws && source install/setup.bash && ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'"
            ;;
        6)
            echo "Available topics:"
            docker exec ros2_sim_container bash -c "source /opt/ros/iron/setup.bash && cd /root/ros2_ws && source install/setup.bash && ros2 topic list"
            ;;
        7)
            echo "Launching RViz..."
            docker exec -it ros2_sim_container bash -c "source /opt/ros/iron/setup.bash && cd /root/ros2_ws && source install/setup.bash && ros2 launch my_robot_bringup robot_rviz.launch.py" &
            ;;
        8)
            echo "Exiting..."
            break
            ;;
        *)
            echo "Invalid choice. Please enter 1-8."
            ;;
    esac
    echo ""
done
