# Startup Scripts Update

**Date:** 2025-11-11  
**Status:** Complete

## Overview

Updated `run.sh` and `start_robot.sh` to clearly indicate that the Web Interface (port 8000) is automatically started and is the primary control interface for the robot.

## Changes Made

### 1. run.sh

**File:** `/home/azzar/project/robotic/lks_robot_project/run.sh`

#### Development Mode Output (--dev)

**Before:**
```
Development setup complete! Access your robot system at:
   n8n Interface: http://localhost:5678
   Robot API: http://localhost:5000
   Sensor Data: curl http://localhost:5000/api/robot/sensors
```

**After:**
```
Development setup complete! Access your robot system at:
   🌐 Web Interface (PRIMARY): http://localhost:8000
   🔧 n8n Workflows (Optional): http://localhost:5678
   🔌 Robot API: http://localhost:5000
   📊 Sensor Data: curl http://localhost:5000/api/robot/sensors

Development Mode Features:
   • Full Web UI with all controls and monitoring
   • Path planning with visual waypoint manager
   • Real-time sensor data display
   • Activity stream for command feedback
   • Simulated ultrasonic sensors (front, back-left, back-right)
   • Simulated line sensor with various patterns
   • Simulated IMU data with realistic variations
   • Simulated LIDAR scan data
   • Fast startup without Gazebo physics simulation
```

#### Production Mode Output

**Before:**
```
Setup complete! Access your robot system at:
   n8n Interface: http://localhost:5678
   Robot Control: http://localhost:5000
```

**After:**
```
Setup complete! Access your robot system at:
   🌐 Web Interface (PRIMARY): http://localhost:8000
   🔧 n8n Workflows (Optional): http://localhost:5678
   🔌 Robot API: http://localhost:5000

Production Mode Features:
   • Full Web UI with complete robot control
   • Path planning and autonomous navigation
   • Real-time sensor monitoring
   • Hardware pinout reference
   • Gazebo physics simulation
   • Complete ROS2 stack
```

---

### 2. start_robot.sh

**File:** `/home/azzar/project/robotic/lks_robot_project/start_robot.sh`

#### Access Information Section

**Before:**
```
Services will be available at:
  🌐 n8n Workflow Interface: http://localhost:5678
  🤖 Robot Web Interface:    http://localhost:5000
  🔌 WebSocket Server:       ws://localhost:8765
```

**After:**
```
Services will be available at:
  🌐 Web Interface (PRIMARY):     http://localhost:8000
  🔧 n8n Workflows (Optional):    http://localhost:5678
  🔌 Robot REST API:              http://localhost:5000
  🔗 WebSocket Server:            ws://localhost:8765

Primary Interface - Web UI (port 8000):
  ✅ Complete robot control (movement, gripper, containers)
  ✅ Real-time sensor monitoring (all sensors)
  ✅ Path planning with visual waypoint manager
  ✅ Activity stream for immediate feedback
  ✅ Hardware pinout reference
  ✅ IMU calibration
  ✅ System status and logs
```

#### Completion Message

**Before:**
```
Development/Production system started successfully!
Sensor simulation active - no Gazebo required / Full Gazebo simulation active
Use './start_robot.sh --status' to check service status
Use './start_robot.sh --logs' to view service logs
Use './start_robot.sh --stop' to stop all services
```

**After:**
```
Development/Production system started successfully!
Sensor simulation active - no Gazebo required / Full Gazebo simulation active

========================================
🌐 Open Web Interface: http://localhost:8000
========================================

Quick Commands:
  Status:  ./start_robot.sh --status
  Logs:    ./start_robot.sh --logs
  Restart: ./start_robot.sh --restart
  Stop:    ./start_robot.sh --stop
```

---

## Technical Details

### Web Interface Auto-Start

The Web Interface is **already included** in the ROS2 launch files and starts automatically when the system boots:

**Launch File:** `ros2_ws/src/my_robot_automation/launch/automation_launch.py`

```python
# Professional web interface (frontend for REST API)
web_robot_interface = Node(
    package='my_robot_automation',
    executable='web_robot_interface.py',
    name='web_robot_interface',
    output='screen',
    parameters=[{
        'use_sim_time': use_sim_time
    }],
    condition=IfCondition(enable_rest_api)
)
```

**Condition:** Web interface starts when `enable_rest_api=true` (default)

**Port:** 8000 (exposed in docker-compose.yml)

### Services Started Automatically

When you run `./run.sh` or `./start_robot.sh`, the following services start:

1. **ROS2 Core** - Robot operating system
2. **REST API Server** - Port 5000 (backend API)
3. **Web Interface** - Port 8000 (PRIMARY UI) ✅
4. **WebSocket Server** - Port 8765 (real-time communication)
5. **N8N Bridge** - Workflow automation integration
6. **Service Servers** - Pick-place, patrol, obstacle avoidance, etc.
7. **N8N** - Port 5678 (optional workflow automation)

---

## User Experience Improvements

### Clear Hierarchy

**Before:** Users might think n8n (port 5678) is the main interface  
**After:** Clearly indicates Web UI (port 8000) is PRIMARY

### Feature Visibility

**Before:** Users need to discover Web UI features  
**After:** Key features listed explicitly in startup output

### Quick Access

**Before:** Multiple URLs without clear priority  
**After:** Primary URL highlighted with emphasis

### Command Reference

**Before:** Full command paths shown  
**After:** Short command reference for quick access

---

## Port Summary

| Port | Service | Priority | Description |
|------|---------|----------|-------------|
| **8000** | **Web Interface** | **PRIMARY** | **Complete robot control and monitoring** |
| 5678 | N8N Workflows | Optional | Workflow automation (advanced) |
| 5000 | REST API | Backend | API endpoints (used by Web UI) |
| 8765 | WebSocket | Backend | Real-time communication |

---

## Usage Examples

### Starting the System

**Development Mode (Fast, No Gazebo):**
```bash
./run.sh --dev
```

**Production Mode (With Gazebo):**
```bash
./run.sh
```

### Using start_robot.sh Directly

**Start with all features:**
```bash
./start_robot.sh
```

**Development mode:**
```bash
./start_robot.sh --dev
```

**Rebuild and start:**
```bash
./start_robot.sh --build
```

**Check status:**
```bash
./start_robot.sh --status
```

**View logs:**
```bash
./start_robot.sh --logs
```

**Restart services:**
```bash
./start_robot.sh --restart
```

**Stop all services:**
```bash
./start_robot.sh --stop
```

---

## Startup Flow

```
1. User runs: ./run.sh
2. Script checks prerequisites (Docker, Docker Compose)
3. Starts Docker containers (ros2-sim, n8n)
4. ROS2 container builds packages
5. Launches automation_launch.py which starts:
   - REST API Server (port 5000)
   - Web Interface (port 8000) ✅
   - WebSocket Server (port 8765)
   - N8N Bridge
   - Service Servers
6. N8N container starts (port 5678)
7. Script displays access information
8. User opens: http://localhost:8000 (PRIMARY)
```

---

## Documentation References

Related documentation for the Web Interface:

- **[Web Interface README](./docs/software/web-interface/README.md)** - Complete Web UI overview
- **[Controls](./docs/software/web-interface/WEB_INTERFACE_CONTROLS.md)** - All available controls
- **[Path Planning](./docs/software/web-interface/WEB_INTERFACE_PATH_PLANNING.md)** - Autonomous navigation
- **[System Architecture](./docs/SYSTEM_ARCHITECTURE.md)** - Overall system design

---

## Benefits

### For New Users

✅ **Clear Starting Point**  
- No confusion about which interface to use
- Primary URL highlighted prominently
- Feature list shows what's available

✅ **Better Onboarding**  
- Understand system capabilities immediately
- See all features in startup message
- Quick command reference

✅ **Reduced Setup Time**  
- Everything starts automatically
- No manual service starting required
- Clear status messages

### For Existing Users

✅ **Improved Workflow**  
- Quick access to primary interface
- Clear port assignments
- Fast command reference

✅ **Better System Understanding**  
- See what's running
- Understand service relationships
- Know what's optional vs required

### For Administrators

✅ **Clear Service Priority**  
- Know which services are critical
- Understand dependencies
- Plan deployment better

✅ **Better Monitoring**  
- Clear status output
- Easy log access
- Quick restart commands

---

## Testing

### Verify Web Interface Starts

1. **Stop any running services:**
   ```bash
   ./start_robot.sh --stop
   ```

2. **Start system:**
   ```bash
   ./run.sh
   ```

3. **Check logs for Web Interface:**
   ```bash
   docker logs ros2_sim_container | grep -i "web_robot_interface"
   ```

4. **Verify Web Interface is accessible:**
   ```bash
   curl http://localhost:8000
   ```

5. **Open in browser:**
   ```
   http://localhost:8000
   ```

### Expected Output

**Console:**
```
🌐 Web Interface (PRIMARY): http://localhost:8000
```

**Browser:**
- Web UI loads successfully
- All tabs visible (Movement, Gripper, Containers, Status, Path Planning, Sensors, Hardware)
- Activity stream shows "Web interface loaded"

---

## Troubleshooting

### Web Interface Not Accessible

**Check if it's running:**
```bash
docker exec -it ros2_sim_container ps aux | grep web_robot_interface
```

**Check logs:**
```bash
docker logs ros2_sim_container | grep -i web
```

**Restart ROS2 container:**
```bash
docker compose restart ros2-sim
```

### Port 8000 Already in Use

**Find process using port:**
```bash
sudo lsof -i :8000
```

**Kill the process or change port in docker-compose.yml**

---

## Summary

### Changes

✅ **run.sh** - Updated development and production mode outputs  
✅ **start_robot.sh** - Enhanced access information and completion messages  
✅ **Emphasis on Web UI** - Port 8000 marked as PRIMARY  
✅ **Feature Listing** - Web UI capabilities shown in startup  
✅ **Quick Commands** - Added command reference

### Impact

**Before:**
- Unclear which interface is primary
- Users might open n8n first
- Web UI features not obvious

**After:**
- Web UI clearly marked as PRIMARY
- Port 8000 emphasized
- Features listed explicitly
- Better user experience

### Result

**The Web Interface (port 8000) is now clearly the primary control interface, with full feature visibility at startup!** 🌐

