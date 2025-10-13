# API Documentation

This documentation provides comprehensive information about the available APIs for controlling and monitoring the LKS Robot Project system.

## API Overview

The robot exposes multiple API interfaces for different use cases:

### Available Interfaces

| Interface | Protocol | Port | Purpose | Documentation |
|-----------|----------|------|---------|---------------|
| **Robot REST API** | HTTP/HTTPS | 5000 | Direct robot control | [Robot API](#robot-rest-api) |
| **n8n Web Interface** | HTTP/HTTPS | 5678 | Workflow automation | [n8n Interface](#n8n-interface) |
| **WebSocket API** | WebSocket | 8765 | Real-time communication | [WebSocket API](#websocket-api) |
| **ROS 2 Topics** | ROS 2 DDS | N/A | Low-level robot control | [ROS 2 Topics](#ros-2-topics) |

## Robot REST API

The robot provides a direct REST API on port 5000 for control and monitoring.

### Basic Movement Control

```bash
# Move robot forward/backward
curl -X POST http://localhost:5000/api/robot/move \
  -H "Content-Type: application/json" \
  -d '{"direction": "forward", "speed": 0.5}'

# Turn robot left/right
curl -X POST http://localhost:5000/api/robot/turn \
  -H "Content-Type: application/json" \
  -d '{"direction": "left", "speed": 0.3}'

# Stop robot
curl -X POST http://localhost:5000/api/robot/stop

# Emergency stop
curl -X POST http://localhost:5000/api/robot/emergency
```

### Actuator Control

```bash
# Control lifter
curl -X POST http://localhost:5000/api/robot/lifter \
  -H "Content-Type: application/json" \
  -d '{"action": "up", "speed": 0.5}'

# Control individual servo
curl -X POST http://localhost:5000/api/robot/servo \
  -H "Content-Type: application/json" \
  -d '{"servo": 1, "angle": 90}'

# Control all servos
curl -X POST http://localhost:5000/api/robot/servos \
  -H "Content-Type: application/json" \
  -d '{"action": "home"}'
```

### Status Monitoring

```bash
# Get robot status
curl http://localhost:5000/api/robot/status
```

## n8n Interface

The n8n workflow automation interface is available at http://localhost:5678 and provides:

- Visual workflow designer
- Pre-configured robot control workflows
- HTTP webhook triggers for external integration
- Scheduled automation tasks

## WebSocket API

Real-time communication interface available on port 8765.

### JavaScript Client Example

```javascript
// WebSocket connection for real-time control
const ws = new WebSocket('ws://localhost:8765');

ws.onopen = function() {
  console.log('Connected to robot');
  
  // Send velocity command
  ws.send(JSON.stringify({
    type: 'velocity_command',
    linear: { x: 0.5, y: 0.0, z: 0.0 },
    angular: { x: 0.0, y: 0.0, z: 0.0 }
  }));
};

ws.onmessage = function(event) {
  const data = JSON.parse(event.data);
  console.log('Sensor data:', data);
};
```

## ROS 2 Topics

Low-level robot control through ROS 2 topics.

### Command Topics

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Robot velocity commands |
| `/lifter_cmd` | `std_msgs/Float32` | Lifter position commands |
| `/servo_cmd` | `sensor_msgs/JointState` | Servo position commands |

### Sensor Topics

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/scan` | `sensor_msgs/LaserScan` | LIDAR scan data |
| `/ultrasonic_*` | `sensor_msgs/Range` | Ultrasonic sensor data |
| `/ir_*` | `sensor_msgs/Range` | Infrared sensor data |
| `/line_sensor` | `std_msgs/Float32` | Line sensor data |

### Python Example

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
    def move_forward(self, speed=0.5):
        twist = Twist()
        twist.linear.x = speed
        self.publisher.publish(twist)

def main():
    rclpy.init()
    controller = RobotController()
    controller.move_forward(0.5)
    rclpy.shutdown()
```

## Error Handling

### HTTP Status Codes

| Code | Description |
|------|-------------|
| `200` | Success |
| `400` | Bad Request |
| `404` | Not Found |
| `500` | Internal Server Error |

### Response Format

```json
{
  "success": true,
  "message": "Robot moved forward at 0.5 m/s"
}
```

## Integration Examples

### Python Integration

```python
import requests

class RobotAPI:
    def __init__(self, base_url='http://localhost:5000'):
        self.base_url = base_url

    def move_forward(self, speed=0.5):
        response = requests.post(
            f'{self.base_url}/api/robot/move',
            json={'direction': 'forward', 'speed': speed}
        )
        return response.json()

    def get_status(self):
        response = requests.get(f'{self.base_url}/api/robot/status')
        return response.json()
```

## Support

For API integration questions:
- Review the specific API documentation sections
- Check the [Troubleshooting Guide](../troubleshooting/)
- Test with the provided examples first
