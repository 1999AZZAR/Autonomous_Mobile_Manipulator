#!/bin/bash

# Simple launcher for Autonomous Mobile Manipulator
# Run with --dev flag for development mode with sensor simulation

if [ "$1" = "--dev" ] || [ "$1" = "-d" ]; then
    echo "Starting Autonomous Mobile Manipulator (Development Mode with Sensor Simulation)..."
    echo ""
    echo "This mode provides simulated sensor data for testing n8n workflows"
    echo "and ROS2 APIs without requiring Gazebo simulation."
    echo ""

    # Run development mode
    ./start_robot.sh --dev

    echo ""
    echo "Development setup complete! Access your robot system at:"
    echo "   n8n Interface: http://localhost:5678"
    echo "   Robot API: http://localhost:5000"
    echo "   Sensor Data: curl http://localhost:5000/api/robot/sensors"
    echo ""
    echo "Development Mode Features:"
    echo "   • Simulated ultrasonic sensors (front, back-left, back-right)"
    echo "   • Simulated line sensor with various patterns"
    echo "   • Simulated IMU data with realistic variations"
    echo "   • Simulated LIDAR scan data"
    echo "   • Fast startup without Gazebo physics simulation"
else
    echo "Starting Autonomous Mobile Manipulator (Production Mode with Gazebo)..."
    echo ""

    # Run the main startup script with default options
    ./start_robot.sh

    echo ""
    echo "Setup complete! Access your robot system at:"
    echo "   n8n Interface: http://localhost:5678"
    echo "   Robot Control: http://localhost:5000"
fi
