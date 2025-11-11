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
    echo "   🌐 Web Interface (PRIMARY): http://localhost:8000"
    echo "   🔧 n8n Workflows (Optional): http://localhost:5678"
    echo "   🔌 Robot API: http://localhost:5000"
    echo "   📊 Sensor Data: curl http://localhost:5000/api/robot/sensors"
    echo ""
    echo "Development Mode Features:"
    echo "   • Full Web UI with all controls and monitoring"
    echo "   • Path planning with visual waypoint manager"
    echo "   • Real-time sensor data display"
    echo "   • Activity stream for command feedback"
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
    echo "   🌐 Web Interface (PRIMARY): http://localhost:8000"
    echo "   🔧 n8n Workflows (Optional): http://localhost:5678"
    echo "   🔌 Robot API: http://localhost:5000"
    echo ""
    echo "Production Mode Features:"
    echo "   • Full Web UI with complete robot control"
    echo "   • Path planning and autonomous navigation"
    echo "   • Real-time sensor monitoring"
    echo "   • Hardware pinout reference"
    echo "   • Gazebo physics simulation"
    echo "   • Complete ROS2 stack"
fi
