#!/bin/bash
# Build the ROS2 workspace

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
echo -e "${BLUE}  Building ROS2 Workspace${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# Check if ros2_ws exists
if [ ! -d "$ROS2_WS" ]; then
    echo -e "${RED}ERROR: ros2_ws directory not found at: $ROS2_WS${NC}"
    exit 1
fi

cd "$ROS2_WS"

# Check ROS2 installation
echo -e "${BLUE}Checking ROS2 installation...${NC}"
if [ -f "/opt/ros/humble/setup.bash" ]; then
    echo -e "${GREEN}✓ ROS2 Humble found${NC}"
    source /opt/ros/humble/setup.bash
elif [ -f "/opt/ros/foxy/setup.bash" ]; then
    echo -e "${GREEN}✓ ROS2 Foxy found${NC}"
    source /opt/ros/foxy/setup.bash
else
    echo -e "${YELLOW}⚠ ROS2 not found in /opt/ros/${NC}"
    echo -e "  Attempting to build anyway..."
fi

# Check colcon
echo -e "${BLUE}Checking colcon...${NC}"
if ! command -v colcon &> /dev/null; then
    echo -e "${RED}✗ colcon not found${NC}"
    echo ""
    echo -e "${YELLOW}Install colcon with:${NC}"
    echo -e "  sudo apt update"
    echo -e "  sudo apt install python3-colcon-common-extensions"
    exit 1
fi
echo -e "${GREEN}✓ colcon found${NC}"

# Check Python dependencies
echo -e "${BLUE}Checking Python dependencies...${NC}"
MISSING_DEPS=()

if ! python3 -c "import flask" 2>/dev/null; then
    MISSING_DEPS+=("flask")
fi

if ! python3 -c "import requests" 2>/dev/null; then
    MISSING_DEPS+=("requests")
fi

if ! python3 -c "import mpu6050" 2>/dev/null; then
    MISSING_DEPS+=("mpu6050-raspberrypi")
fi

if [ ${#MISSING_DEPS[@]} -gt 0 ]; then
    echo -e "${YELLOW}⚠ Missing Python packages: ${MISSING_DEPS[*]}${NC}"
    echo -e "${BLUE}Installing missing packages...${NC}"
    pip3 install ${MISSING_DEPS[*]}
else
    echo -e "${GREEN}✓ All Python dependencies installed${NC}"
fi

echo ""
echo -e "${BLUE}Building workspace...${NC}"
echo -e "  This may take a few minutes on first build"
echo ""

# Build with colcon
colcon build

if [ $? -eq 0 ]; then
    echo ""
    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN}✓ Workspace built successfully!${NC}"
    echo -e "${GREEN}========================================${NC}"
    echo ""
    echo -e "${BLUE}To use the workspace:${NC}"
    echo -e "  cd ros2_ws"
    echo -e "  source install/setup.bash"
    echo ""
    echo -e "${BLUE}To start the robot:${NC}"
    echo -e "  ./start_hardware.sh     # For real hardware"
    echo -e "  ./start_simulation.sh   # For simulation"
else
    echo ""
    echo -e "${RED}========================================${NC}"
    echo -e "${RED}✗ Build failed${NC}"
    echo -e "${RED}========================================${NC}"
    echo ""
    echo -e "${YELLOW}Check the error messages above${NC}"
    exit 1
fi

