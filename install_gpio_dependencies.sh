#!/bin/bash

# GPIO Dependencies Installation Script for Raspberry Pi
# This script installs all necessary GPIO libraries and services

set -e

echo "========================================"
echo "GPIO Dependencies Installation"
echo "========================================"
echo ""

# Check if running on Raspberry Pi
if ! grep -q "Raspberry Pi" /proc/cpuinfo; then
    echo "WARNING: This script is designed for Raspberry Pi."
    echo "Continue anyway? (y/n)"
    read -r response
    if [[ "$response" != "y" ]]; then
        echo "Installation cancelled."
        exit 1
    fi
fi

echo "Updating package list..."
sudo apt update

echo ""
echo "Installing GPIO libraries..."

# Install pigpio (hardware PWM daemon)
echo "Installing pigpio..."
sudo apt install -y pigpio python3-pigpio

# Install gpiozero (high-level GPIO library)
echo "Installing gpiozero..."
sudo apt install -y python3-gpiozero

# Install RPi.GPIO (low-level GPIO access)
echo "Installing RPi.GPIO..."
sudo apt install -y python3-rpi.gpio

# Install additional dependencies
echo "Installing additional dependencies..."
sudo apt install -y python3-smbus2 python3-spidev

echo ""
echo "Starting and enabling pigpiod service..."

# Start pigpiod service
sudo systemctl start pigpiod

# Enable pigpiod to start on boot
sudo systemctl enable pigpiod

# Check if user is in gpio group
echo ""
echo "Checking GPIO group membership..."
if groups $USER | grep -q gpio; then
    echo "✓ User $USER is already in gpio group"
else
    echo "Adding user $USER to gpio group..."
    sudo usermod -a -G gpio $USER
    echo "✓ User added to gpio group"
    echo "NOTE: You may need to log out and back in for group changes to take effect"
fi

echo ""
echo "Verifying installation..."

# Check if pigpiod is running
if sudo systemctl is-active --quiet pigpiod; then
    echo "✓ pigpiod service is running"
else
    echo "✗ pigpiod service is not running"
    echo "Try: sudo systemctl start pigpiod"
fi

# Test Python imports
echo ""
echo "Testing Python imports..."
python3 -c "import gpiozero; print('✓ gpiozero imported successfully')" 2>/dev/null || echo "✗ gpiozero import failed"
python3 -c "import pigpio; print('✓ pigpio imported successfully')" 2>/dev/null || echo "✗ pigpio import failed"
python3 -c "import RPi.GPIO as GPIO; print('✓ RPi.GPIO imported successfully')" 2>/dev/null || echo "✗ RPi.GPIO import failed"

echo ""
echo "========================================"
echo "Installation Complete!"
echo "========================================"
echo ""
echo "Next steps:"
echo "1. Log out and back in (if you were added to gpio group)"
echo "2. Run GPIO test: python3 test_gpio.py"
echo "3. Start the web interface: python3 web_robot_interface.py"
echo ""
echo "For Docker usage, ensure container has GPIO access:"
echo "  --device /dev/gpiomem --device /dev/mem --privileged"
echo "  -v /run/pigpio:/run/pigpio"
echo ""

echo "Installation completed successfully!"
