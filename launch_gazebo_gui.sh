#!/bin/bash

# Script to launch Gazebo GUI for robot simulation
# This script should be run on the host system

echo "Starting Gazebo GUI for robot simulation..."

# Check if Docker container is running
if ! docker ps | grep -q ros2_sim_container; then
    echo "Error: ros2_sim_container is not running. Please start it first with:"
    echo "docker compose up -d ros2-sim"
    exit 1
fi

# Set environment for GUI
export DISPLAY=${DISPLAY:-:0}
export QT_QPA_PLATFORM=wayland

# Launch Gazebo client from container
echo "Launching Gazebo GUI..."
docker exec -it ros2_sim_container bash -c "
    source /opt/ros/iron/setup.bash && 
    cd /root/ros2_ws && 
    source install/setup.bash && 
    gzclient --verbose
"

echo "Gazebo GUI launched!"
