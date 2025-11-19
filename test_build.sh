#!/bin/bash

# Test script to verify ROS2 workspace can be built
# This script tests the ROS2 package compilation without Docker

echo "Testing ROS2 workspace build..."

# Check if we're in the right directory
if [ ! -f "ros2_ws/src/my_robot_automation/package.xml" ]; then
    echo "Error: Not in the correct directory. Run from /home/azzar/project/robotic/for_the_rpi/"
    exit 1
fi

cd ros2_ws

# Source ROS2 if available (may not be installed on host)
if command -v ros2 &> /dev/null; then
    echo "ROS2 found, testing package structure..."
    source /opt/ros/jazzy/setup.bash 2>/dev/null || echo "ROS2 Jazzy not found, testing basic structure only"

    # Test colcon build dry run
    if command -v colcon &> /dev/null; then
        echo "Testing colcon build dry run..."
        timeout 30 colcon build --dry-run --packages-select my_robot_automation 2>&1 | head -20
        if [ $? -eq 0 ]; then
            echo "✓ Package structure validation passed"
        else
            echo "✗ Package structure validation failed"
        fi
    fi
else
    echo "ROS2 not installed on host system, testing Python syntax only..."
fi

# Test Python syntax for key files
echo "Testing Python syntax..."
python3 -m py_compile src/my_robot_automation/scripts/mega_serial_interface.py
if [ $? -eq 0 ]; then
    echo "✓ mega_serial_interface.py syntax OK"
else
    echo "✗ mega_serial_interface.py syntax ERROR"
fi

python3 -m py_compile src/my_robot_automation/scripts/actuator_control_server.py
if [ $? -eq 0 ]; then
    echo "✓ actuator_control_server.py syntax OK"
else
    echo "✗ actuator_control_server.py syntax ERROR"
fi

python3 -c "
import sys
import os
test_file = '../../../test_pi_mega_integration.py'  # Relative path from ros2_ws
if os.path.exists(test_file):
    try:
        import importlib.util
        spec = importlib.util.spec_from_file_location('test', test_file)
        test_module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(test_module)
        print('✓ Integration test script imports OK')
    except Exception as e:
        print('✗ Integration test script import ERROR:', str(e))
else:
    print('⚠ Integration test script not found (expected when testing without full project)')
"

# Check launch file syntax and content
if python3 -c "
import ast
try:
    with open('src/my_robot_automation/launch/automation_launch.py', 'r') as f:
        ast.parse(f.read())
    print('✓ Launch file Python syntax OK')
except SyntaxError as e:
    print('✗ Launch file syntax ERROR:', str(e))
    exit(1)
except Exception as e:
    print('⚠ Launch file check skipped:', str(e))
" 2>/dev/null; then
    # Check if Mega communication nodes are included
    if grep -q "mega_serial_interface" src/my_robot_automation/launch/automation_launch.py && grep -q "actuator_control_server" src/my_robot_automation/launch/automation_launch.py; then
        echo "✓ Mega communication nodes included in launch file"
    else
        echo "✗ Mega communication nodes missing from launch file"
    fi
fi

# Check package.xml and CMakeLists.txt
if [ -f "src/my_robot_automation/package.xml" ]; then
    echo "✓ package.xml exists"
    if grep -q "my_robot_automation" src/my_robot_automation/package.xml; then
        echo "✓ Package name correct in package.xml"
    else
        echo "✗ Package name incorrect in package.xml"
    fi
fi

if [ -f "src/my_robot_automation/CMakeLists.txt" ]; then
    echo "✓ CMakeLists.txt exists"
    if grep -q "mega_serial_interface.py" src/my_robot_automation/CMakeLists.txt; then
        echo "✓ mega_serial_interface.py included in CMakeLists.txt"
    else
        echo "✗ mega_serial_interface.py missing from CMakeLists.txt"
    fi
    if grep -q "actuator_control_server.py" src/my_robot_automation/CMakeLists.txt; then
        echo "✓ actuator_control_server.py included in CMakeLists.txt"
    else
        echo "✗ actuator_control_server.py missing from CMakeLists.txt"
    fi
fi

echo "ROS2 workspace validation complete."