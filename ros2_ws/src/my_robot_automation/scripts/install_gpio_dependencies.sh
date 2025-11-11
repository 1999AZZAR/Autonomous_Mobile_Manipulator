#!/bin/bash

# Installation script for GPIO control dependencies
# Run this on the Raspberry Pi 5

echo "=========================================="
echo "Installing GPIO Control Dependencies"
echo "=========================================="
echo ""

# Update package lists
echo "[1/5] Updating package lists..."
sudo apt update

# Install pigpio daemon for hardware PWM
echo "[2/5] Installing pigpio daemon..."
sudo apt install -y pigpio python3-pigpio

# Install gpiozero
echo "[3/5] Installing gpiozero..."
sudo apt install -y python3-gpiozero

# Install RPi.GPIO
echo "[4/5] Installing RPi.GPIO..."
sudo apt install -y python3-rpi.gpio

# Enable and start pigpio daemon
echo "[5/5] Enabling pigpio daemon..."
sudo systemctl enable pigpiod
sudo systemctl start pigpiod

echo ""
echo "=========================================="
echo "Installation Complete!"
echo "=========================================="
echo ""
echo "GPIO control dependencies installed:"
echo "  - pigpio daemon (hardware PWM)"
echo "  - gpiozero library"
echo "  - RPi.GPIO library"
echo ""
echo "The pigpio daemon has been started and enabled."
echo ""
echo "You can now run the web interface with GPIO control:"
echo "  python3 web_robot_interface.py"
echo ""
echo "To test GPIO functionality, try:"
echo "  python3 test_gpio.py"
echo ""

