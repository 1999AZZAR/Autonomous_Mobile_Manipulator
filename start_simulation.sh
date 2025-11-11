#!/bin/bash
# Start the robot web interface in SIMULATION mode (no hardware required)
# Use this script for testing without physical hardware

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
echo -e "${BLUE}  Starting Robot in SIMULATION Mode${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# Check if ros2_ws exists
if [ ! -d "$ROS2_WS" ]; then
    echo -e "${RED}ERROR: ros2_ws directory not found at: $ROS2_WS${NC}"
    exit 1
fi

echo -e "${YELLOW}⚠ Simulation Mode${NC}"
echo -e "  All sensor data will be simulated"
echo -e "  No hardware required"
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

# Start web interface with --simulation flag
echo -e "${BLUE}Starting web interface in SIMULATION mode...${NC}"
echo ""

python3 src/my_robot_automation/scripts/web_robot_interface.py --simulation

# If we get here, the server stopped
echo ""
echo -e "${YELLOW}Web interface stopped${NC}"

