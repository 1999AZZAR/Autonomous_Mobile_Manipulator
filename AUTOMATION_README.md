# Robot Automation System

This document describes the complete ROS2 automation system that has been implemented to provide reliable robot control and API services.

## System Architecture

The automation system consists of several key components:

### 1. Core Automation Services (`my_robot_automation`)

**Main Components:**
- `robot_automation_server.py` - Central automation coordinator
- `rest_api_server.py` - HTTP REST API server (port 5678)
- `websocket_server.py` - Real-time WebSocket communication (port 8765)
- `n8n_ros2_bridge.py` - Bridge between n8n workflows and ROS2 services

**Services Provided:**
- Pick and place operations
- Patrol navigation
- Obstacle avoidance
- Emergency stop handling
- Robot status monitoring
- Mode management (AUTONOMOUS, MANUAL, EMERGENCY, MAINTENANCE)

### 2. Manipulation System (`my_robot_manipulation`)

**Components:**
- MoveIt2 configuration for arm control
- Gripper control systems
- Pick and place execution
- Collision avoidance for manipulation

**Configuration Files:**
- `arm_planning_group.yaml` - Arm planning group configuration
- `arm_servo.yaml` - Servo control parameters
- `ompl_planning.yaml` - Motion planning algorithms

### 3. Navigation System (`my_robot_navigation`)

**Components:**
- Navigation2 integration
- Path planning and execution
- Obstacle avoidance
- Patrol route management

**Configuration Files:**
- `nav2_params.yaml` - Complete Navigation2 parameter set

## API Endpoints

### REST API (Port 5678)

#### Robot Control
```bash
# Get robot status
GET /api/robot/status

# Set robot mode
POST /api/robot/mode
{
  "mode": "AUTONOMOUS",
  "reason": "Starting autonomous operation"
}

# Emergency stop
POST /api/robot/emergency-stop
{
  "activate": true,
  "reason": "Manual emergency stop"
}
```

#### Automation Operations
```bash
# Pick and place
POST /api/robot/pick-place
{
  "pickup_location": {"x": 1.0, "y": 0.5, "z": 0.0},
  "place_location": {"x": -1.0, "y": 0.5, "z": 0.0},
  "object_type": "cube",
  "timeout_seconds": 60
}

# Patrol
POST /api/robot/patrol
{
  "waypoints": [
    {"x": 1.0, "y": 0.0, "z": 0.0},
    {"x": 1.0, "y": 1.0, "z": 0.0},
    {"x": 0.0, "y": 1.0, "z": 0.0}
  ],
  "patrol_cycles": 2,
  "patrol_speed": 0.5
}

# Obstacle avoidance navigation
POST /api/robot/obstacle-avoidance
{
  "target_location": {"x": 2.0, "y": 1.0, "z": 0.0},
  "avoidance_distance": 0.5,
  "max_speed": 0.5
}
```

### WebSocket API (Port 8765)

Real-time bidirectional communication for:
- Robot status updates
- Safety status monitoring
- Velocity commands
- Laser scan data
- Odometry information

**Message Types:**
```javascript
// Send velocity command
{
  "type": "velocity_command",
  "data": {
    "linear_x": 0.5,
    "linear_y": 0.0,
    "angular_z": 0.0
  }
}

// Receive robot status
{
  "type": "robot_status",
  "timestamp": "2024-01-01T12:00:00",
  "data": {
    "mode": "AUTONOMOUS",
    "state": "IDLE",
    "battery_level": 85.0,
    "current_pose": {...}
  }
}
```

### n8n Webhook Endpoints

Compatible with existing n8n workflows:

```bash
# Basic robot control
POST /webhook/robot-control
{
  "command": "forward",
  "speed": 0.5
}

# Emergency stop
POST /webhook/emergency-stop
{
  "emergency": true,
  "reason": "Workflow triggered stop"
}

# Pick and place
POST /webhook/robot/pick_place
{
  "pickup_location": {"x": 1.0, "y": 0.5},
  "place_location": {"x": -1.0, "y": 0.5}
}
```

## Safety Systems

### Emergency Stop
- Multiple activation methods (API, WebSocket, n8n webhook)
- Immediate robot halt
- Safety status monitoring
- Recovery procedures

### Obstacle Detection
- Real-time laser scan monitoring
- Minimum distance thresholds
- Dynamic avoidance
- Safety warnings

### State Management
- Robot mode tracking (AUTONOMOUS, MANUAL, EMERGENCY, MAINTENANCE)
- Task state monitoring
- Error handling and recovery
- System health diagnostics

## Custom Messages and Services

### Messages
- `RobotStatus.msg` - Complete robot state information
- `AutomationTask.msg` - Task execution status
- `SafetyStatus.msg` - Safety system status

### Services
- `ExecutePickPlace.srv` - Pick and place operations
- `ExecutePatrol.srv` - Patrol navigation
- `ExecuteObstacleAvoidance.srv` - Obstacle avoidance navigation
- `GetRobotStatus.srv` - Status retrieval
- `EmergencyStop.srv` - Emergency stop control
- `SetRobotMode.srv` - Mode management

## Launch System

### Complete Automation Launch
```bash
ros2 launch my_robot_automation automation_launch.py
```

**Launch Arguments:**
- `use_sim_time:=true/false` - Simulation time usage
- `enable_rest_api:=true/false` - Enable REST API server
- `enable_websocket:=true/false` - Enable WebSocket server
- `enable_n8n_bridge:=true/false` - Enable n8n bridge

## Docker Integration

The system is fully integrated with Docker Compose:

```yaml
services:
  ros2-sim:
    # Builds and launches complete automation system
    command: >
      bash -c "
      source /opt/ros/iron/setup.bash &&
      cd /root/ros2_ws &&
      colcon build --packages-select my_robot_automation my_robot_manipulation my_robot_navigation &&
      source install/setup.bash &&
      ros2 launch my_robot_automation automation_launch.py use_sim_time:=true
      "
    ports:
      - "5678:5678" # REST API
      - "8765:8765" # WebSocket
```

## Usage Examples

### 1. Start the System
```bash
docker-compose up
```

### 2. Test REST API
```bash
# Check system health
curl http://localhost:5678/health

# Get robot status
curl http://localhost:5678/api/robot/status

# Execute pick and place
curl -X POST http://localhost:5678/api/robot/pick-place \
  -H "Content-Type: application/json" \
  -d '{
    "pickup_location": {"x": 1.0, "y": 0.5, "z": 0.0},
    "place_location": {"x": -1.0, "y": 0.5, "z": 0.0},
    "object_type": "cube"
  }'
```

### 3. WebSocket Connection
```javascript
const ws = new WebSocket('ws://localhost:8765');

ws.onopen = function() {
  console.log('Connected to robot');
  
  // Send velocity command
  ws.send(JSON.stringify({
    type: 'velocity_command',
    data: { linear_x: 0.5, angular_z: 0.0 }
  }));
};

ws.onmessage = function(event) {
  const data = JSON.parse(event.data);
  console.log('Received:', data);
};
```

### 4. n8n Integration
The system is fully compatible with existing n8n workflows. Simply update webhook URLs to point to `http://localhost:5678/webhook/` endpoints.

## Monitoring and Diagnostics

### Robot Status Monitoring
- Real-time status updates via WebSocket
- REST API status queries
- Safety system monitoring
- Task execution tracking

### System Health
- CPU and memory usage monitoring
- Battery level tracking
- Communication status
- Error logging and reporting

## Error Handling

### Robust Error Recovery
- Service call timeouts
- Connection failure handling
- Task execution error recovery
- Safety system activation

### Logging
- Comprehensive logging at all levels
- Error tracking and reporting
- Performance monitoring
- Debug information

## Development and Testing

### Building the System
```bash
cd ros2_ws
colcon build --packages-select my_robot_automation my_robot_manipulation my_robot_navigation
source install/setup.bash
```

### Testing Individual Components
```bash
# Test automation server
ros2 run my_robot_automation robot_automation_server.py

# Test REST API
ros2 run my_robot_automation rest_api_server.py

# Test WebSocket server
ros2 run my_robot_automation websocket_server.py
```

This automation system provides a complete, reliable foundation for autonomous robot operations with multiple interfaces for integration and control.
