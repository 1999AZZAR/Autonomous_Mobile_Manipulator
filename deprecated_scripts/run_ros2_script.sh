#!/bin/bash
# Script to properly run ROS2 scripts with sourced environment

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
ROS2_WS="$SCRIPT_DIR/ros2_ws"

if [ ! -d "$ROS2_WS" ]; then
    echo "ERROR: ros2_ws directory not found at: $ROS2_WS"
    echo "Please run this script from the project root directory"
    exit 1
fi

echo "Project root: $SCRIPT_DIR"
echo "ROS2 workspace: $ROS2_WS"
echo "Sourcing ROS2 workspace..."

cd "$ROS2_WS"
source install/setup.bash

echo "Running ROS2 script: $@"
python3 "$@"

