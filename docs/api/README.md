# API Reference

REST API on port 8001. All endpoints return JSON. WebSocket on the same port for real-time sensor data.

## Base URL

```
http://localhost:8001
```

## Authentication

None. The API is designed for local network use on the robot. No auth tokens required.

## Response Format

Success:
```json
{
  "success": true,
  "message": "Robot moved forward",
  "data": { ... }
}
```

Error:
```json
{
  "success": false,
  "error": "Invalid direction"
}
```

## Robot Control

### Movement

```bash
# Move forward/backward/strafe
curl -X POST http://localhost:8001/api/robot/move \
  -H "Content-Type: application/json" \
  -d '{"direction": "forward", "speed": 0.5}'

# Turn left/right
curl -X POST http://localhost:8001/api/robot/turn \
  -H "Content-Type: application/json" \
  -d '{"direction": "left", "speed": 0.3}'

# Stop
curl -X POST http://localhost:8001/api/robot/stop

# Emergency stop
curl -X POST http://localhost:8001/api/robot/emergency-stop
```

### Speed

```bash
# Set speed multiplier (0.0 - 1.0)
curl -X POST http://localhost:8001/api/robot/speed \
  -H "Content-Type: application/json" \
  -d '{"speed": 0.8}'

# Toggle turbo
curl -X POST http://localhost:8001/api/robot/turbo
```

### Manipulator

```bash
# Gripper open/close
curl -X POST http://localhost:8001/api/robot/picker/gripper \
  -H "Content-Type: application/json" \
  -d '{"action": "open"}'

# Tilt angle (0-180)
curl -X POST http://localhost:8001/api/robot/picker/gripper_tilt \
  -H "Content-Type: application/json" \
  -d '{"angle": 90}'

# Lifter up/down
curl -X POST http://localhost:8001/api/robot/lifter/up
curl -X POST http://localhost:8001/api/robot/lifter/down
```

### Individual Wheels

```bash
# Control specific wheel (id: 1-4, speed: -100 to 100)
curl -X POST http://localhost:8001/api/robot/wheels/2 \
  -H "Content-Type: application/json" \
  -d '{"speed": 75}'

# Stop all wheels
curl -X POST http://localhost:8001/api/robot/wheels/stop
```

## Sensor Data

```bash
# All sensors (flat keys for frontend)
curl http://localhost:8001/api/robot/sensors

# Position + sensors
curl http://localhost:8001/api/robot/position

# Sensor diagnostics
curl http://localhost:8001/api/robot/sensors/diagnostics

# Raw sensor feed
curl http://localhost:8001/api/feeds
```

### Sensor Response Keys

```json
{
  "laser_left_front": 245,
  "laser_left_back": 312,
  "laser_right_front": 289,
  "laser_right_back": 267,
  "laser_back_left": 198,
  "laser_back_right": 210,
  "ultra_front_left": 250,
  "ultra_front_right": 300,
  "line_left": 234,
  "line_center": 678,
  "line_right": 345,
  "imu_heading": 45.2,
  "motor_status": { "m1": 0, "m2": 60, "m3": 60, "m4": 60 }
}
```

## Safety

```bash
# Enable/disable virtual bumper
curl -X POST http://localhost:8001/api/robot/safety/perimeter \
  -H "Content-Type: application/json" \
  -d '{"enabled": true}'

# Check limit switches
curl http://localhost:8001/api/robot/safety/limit-switches
```

## Sequences

```bash
# Save a movement sequence
curl -X POST http://localhost:8001/api/robot/sequences/save \
  -H "Content-Type: application/json" \
  -d '{"name": "patrol_1", "sequence": ["f", "f", "c", "f", "s"]}'

# Execute
curl -X POST http://localhost:8001/api/robot/sequences/execute \
  -H "Content-Type: application/json" \
  -d '{"name": "patrol_1"}'

# List saved
curl http://localhost:8001/api/robot/sequences/list
```

## Automation Rules

```bash
# List rules
curl http://localhost:8001/api/automations

# Create rule
curl -X POST http://localhost:8001/api/automations \
  -H "Content-Type: application/json" \
  -d '{
    "name": "Wall Following",
    "trigger_type": "sensor",
    "trigger_config": {"sensor": "ultra_front_left", "condition": "less_than", "value": 30},
    "action_type": "move",
    "action_config": {"direction": "right", "speed": 0.3}
  }'

# Toggle
curl -X POST http://localhost:8001/api/automations/1/toggle

# Run now
curl -X POST http://localhost:8001/api/automations/1/run
```

## AI Decision Engine

```bash
# Start AI mode (activates camera)
curl -X POST http://localhost:8001/api/ai/start \
  -H "Content-Type: application/json" \
  -d '{"task": "Navigate to kitchen and find cup"}'

# Stop AI (camera turns off)
curl -X POST http://localhost:8001/api/ai/stop

# Status
curl http://localhost:8001/api/ai/status

# Camera snapshot
curl http://localhost:8001/api/ai/camera/snapshot --output snap.jpg

# Decision history
curl http://localhost:8001/api/ai/decisions
```

## Waypoint Memory

```bash
# Save a path
curl -X POST http://localhost:8001/api/waypoints/paths \
  -H "Content-Type: application/json" \
  -d '{"name": "To Kitchen", "waypoints": [{"x": 1.0, "y": 0.5}, {"x": 2.0, "y": 1.0}]}'

# List paths
curl http://localhost:8001/api/waypoints/paths

# Start recording (IMU dead reckoning)
curl -X POST http://localhost:8001/api/waypoints/record

# Stop recording
curl -X POST http://localhost:8001/api/waypoints/stop

# Replay path
curl -X POST http://localhost:8001/api/waypoints/replay/1

# Stop replay
curl -X POST http://localhost:8001/api/waypoints/replay/stop
```

## WebSocket

Connect to `ws://localhost:8001/ws/sensors` for real-time sensor data pushes.

```javascript
const ws = new WebSocket('ws://localhost:8001/ws/sensors');
ws.onmessage = (e) => {
  const data = JSON.parse(e.data);
  // data contains sensor readings at ~10Hz
};
```

## Health Check

```bash
curl http://localhost:8001/health
# {"status": "ok", "timestamp": "2025-01-15T10:30:00Z"}
```
