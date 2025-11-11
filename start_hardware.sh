#!/bin/bash
# Start the robot web interface in HARDWARE mode (real sensors)
# Use this script when running directly on Raspberry Pi with real hardware

set -e

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
ROS2_WS="$SCRIPT_DIR/ros2_ws"

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}  Starting Robot in HARDWARE Mode${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# Check if ros2_ws exists
if [ ! -d "$ROS2_WS" ]; then
    echo -e "${RED}ERROR: ros2_ws directory not found at: $ROS2_WS${NC}"
    exit 1
fi

# Check I2C permissions
echo -e "${BLUE}Checking I2C permissions...${NC}"
if groups | grep -q i2c; then
    echo -e "${GREEN}✓ User is in i2c group${NC}"
else
    echo -e "${YELLOW}⚠ Warning: User not in i2c group${NC}"
    echo -e "  Add with: sudo usermod -a -G i2c \$USER"
    echo -e "  Then logout and login again"
fi

# Check if IMU is detected
echo -e "${BLUE}Checking for MPU6050 IMU...${NC}"
if command -v i2cdetect &> /dev/null; then
    if i2cdetect -y 1 2>/dev/null | grep -q " 68 "; then
        echo -e "${GREEN}✓ MPU6050 detected at 0x68${NC}"
    else
        echo -e "${YELLOW}⚠ Warning: MPU6050 not detected${NC}"
        echo -e "  Check wiring and run: i2cdetect -y 1"
    fi
else
    echo -e "${YELLOW}⚠ i2cdetect not found. Install with: sudo apt-get install i2c-tools${NC}"
fi

echo ""
echo -e "${BLUE}Starting web interface in HARDWARE mode...${NC}"
echo ""

# Navigate to ROS2 workspace
cd "$ROS2_WS"

# Check if workspace is built
if [ -f "install/setup.bash" ]; then
    echo -e "${GREEN}✓ Sourcing ROS2 workspace${NC}"
    source install/setup.bash
else
    echo -e "${RED}✗ ROS2 workspace not built!${NC}"
    echo -e "  The web interface requires ROS2 environment."
    echo ""
    echo -e "${BLUE}Building workspace now (this takes a few minutes)...${NC}"
    
    # Check if colcon is available
    if ! command -v colcon &> /dev/null; then
        echo -e "${RED}ERROR: colcon not found. Install with:${NC}"
        echo -e "  sudo apt update"
        echo -e "  sudo apt install python3-colcon-common-extensions"
        exit 1
    fi
    
    # Build the workspace
    colcon build
    
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✓ Workspace built successfully${NC}"
        source install/setup.bash
    else
        echo -e "${RED}✗ Build failed${NC}"
        echo -e "  Try manually: cd ros2_ws && colcon build"
        exit 1
    fi
fi

# Start web interface with --hardware flag
python3 src/my_robot_automation/scripts/web_robot_interface.py --hardware

# If we get here, the server stopped
echo ""
echo -e "${YELLOW}Web interface stopped${NC}"

