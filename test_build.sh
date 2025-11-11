#!/bin/bash
set -e

echo "Step 1: Source ROS2"
source /opt/ros/jazzy/setup.bash

echo "Step 2: Change to workspace"
cd /root/ros2_ws

echo "Step 3: Build ROS2 packages"
colcon build --packages-select my_robot_automation --cmake-args -DCMAKE_BUILD_TYPE=Release

echo "Step 4: Source install"
source install/setup.bash

echo "Step 5: Test Python imports"
python3 -c "from my_robot_automation.srv import ControlGripper; print('ROS2 imports work!')"

echo "Step 6: Install Python packages"
pip3 install --break-system-packages requests smbus2 mpu6050-raspberrypi spidev lgpio flask flask-cors websockets

echo "Step 7: Start web interface"
exec python3 /root/ros2_ws/src/my_robot_automation/scripts/web_robot_interface.py --hardware
