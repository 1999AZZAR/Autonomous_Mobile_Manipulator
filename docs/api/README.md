# API Documentation

This documentation provides comprehensive information about the available APIs for controlling and monitoring the Autonomous Mobile Manipulator robot.

## API Overview

The robot exposes multiple API interfaces for different use cases:

### Available Interfaces

| Interface | Protocol | Port | Purpose | Documentation |
|-----------|----------|------|---------|---------------|
| **REST API** | HTTP/HTTPS | 5678 | High-level robot control | [REST API](./rest-api.md) |
| **WebSocket API** | WebSocket | 8765 | Real-time communication | [WebSocket API](./websocket.md) |
| **ROS 2 Topics** | ROS 2 DDS | N/A | Low-level robot control | [ROS 2 Topics](./ros2-topics.md) |
| **MQTT API** | MQTT | 1883 | IoT integration | [MQTT API](./mqtt.md) |

## Quick Start Examples

### HTTP REST API

#### Basic Movement Control

```bash
# Move robot forward
curl -X POST http://localhost:5678/webhook/robot-control \
  -H "Content-Type: application/json" \
  -d '{"command": "forward", "speed": 0.5}'

# Rotate robot clockwise
curl -X POST http://localhost:5678/webhook/robot-control \
  -H "Content-Type: application/json" \
  -d '{"command": "rotate_cw", "speed": 1.0}'

# Emergency stop
curl -X POST http://localhost:5678/webhook/emergency-stop \
  -H "Content-Type: application/json" \
  -d '{"emergency": true, "reason": "manual_stop"}'
```

#### Pick and Place Operation

```bash
# Execute pick and place task
curl -X POST http://localhost:5678/webhook/robot/pick_place \
  -H "Content-Type: application/json" \
  -d '{
    "pickup_location": {"x": 1.0, "y": 0.5},
    "place_location": {"x": -1.0, "y": 0.5},
    "object_type": "cube"
  }'
```

### WebSocket Real-time Communication

#### JavaScript Client Example

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

  // Process laser scan, IMU, odometry data
  if (data.type === 'sensor_data') {
    updateDashboard(data);
  }
};
```

### ROS 2 Topic Communication

#### Python ROS 2 Example

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import json

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control_loop)

    def control_loop(self):
        # Create velocity command
        twist = Twist()
        twist.linear.x = 0.5  # Forward movement
        twist.angular.z = 0.0  # No rotation

        self.publisher.publish(twist)
        self.get_logger().info('Sent velocity command')

def main():
    rclpy.init()
    controller = RobotController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## API Endpoints and Topics

### REST API Endpoints

| Method | Endpoint | Description | Parameters |
|--------|----------|-------------|------------|
| `POST` | `/webhook/robot-control` | Basic movement commands | `command`, `speed`, `duration` |
| `POST` | `/webhook/emergency-stop` | Emergency stop activation | `emergency`, `reason` |
| `POST` | `/webhook/robot/pick_place` | Pick and place operations | `pickup_location`, `place_location`, `object_type` |
| `POST` | `/webhook/robot/patrol` | Autonomous patrol activation | `pattern`, `duration` |
| `GET` | `/webhook/robot/status` | Robot status information | None |
| `GET` | `/webhook/robot/sensors` | Current sensor readings | None |

### WebSocket Message Types

#### Command Messages (Client → Robot)

```json
{
  "type": "velocity_command",
  "linear": {
    "x": 0.5,    // Forward/backward velocity (m/s)
    "y": 0.0,    // Lateral velocity (m/s)
    "z": 0.0     // Vertical velocity (m/s)
  },
  "angular": {
    "x": 0.0,    // Roll velocity (rad/s)
    "y": 0.0,    // Pitch velocity (rad/s)
    "z": 1.0     // Yaw velocity (rad/s)
  }
}
```

```json
{
  "type": "arm_command",
  "joint_positions": [0, -1.57, 0, 0, 0, 0],
  "gripper_command": "close"
}
```

#### Response Messages (Robot → Client)

```json
{
  "type": "sensor_data",
  "timestamp": 1640995200.123,
  "laser_scan": {
    "ranges": [2.1, 2.3, 2.0, ...],
    "intensities": [100, 95, 105, ...],
    "angle_min": -3.14159,
    "angle_max": 3.14159,
    "angle_increment": 0.017453
  },
  "imu": {
    "orientation": {
      "w": 0.98, "x": 0.0, "y": 0.0, "z": 0.17
    },
    "angular_velocity": {
      "x": 0.01, "y": 0.02, "z": 0.03
    },
    "linear_acceleration": {
      "x": 0.1, "y": 0.0, "z": 9.8
    }
  },
  "odometry": {
    "pose": {
      "position": {"x": 1.2, "y": 0.8, "z": 0.0},
      "orientation": {"w": 0.98, "x": 0.0, "y": 0.0, "z": 0.17}
    },
    "twist": {
      "linear": {"x": 0.1, "y": 0.0, "z": 0.0},
      "angular": {"x": 0.0, "y": 0.0, "z": 0.05}
    }
  }
}
```

### ROS 2 Topics

#### Command Topics (Publisher)

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Robot velocity commands |
| `/arm_controller/commands` | `trajectory_msgs/JointTrajectory` | Arm joint commands |
| `/gripper_controller/commands` | `control_msgs/GripperCommand` | Gripper control |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | Initial pose setting |
| `/goal_pose` | `geometry_msgs/PoseStamped` | Navigation goals |

#### Sensor Topics (Subscriber)

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/scan` | `sensor_msgs/LaserScan` | LiDAR scan data |
| `/imu/data` | `sensor_msgs/Imu` | IMU sensor data |
| `/camera/image_raw` | `sensor_msgs/Image` | Camera image stream |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | Camera calibration data |
| `/joint_states` | `sensor_msgs/JointState` | Robot joint positions |
| `/odom` | `nav_msgs/Odometry` | Robot odometry data |
| `/battery_state` | `sensor_msgs/BatteryState` | Battery status |

#### Status Topics (Subscriber)

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | System diagnostics |
| `/robot_status` | `std_msgs/String` | Robot operational status |
| `/emergency_status` | `std_msgs/Bool` | Emergency stop status |
| `/mission_status` | `std_msgs/String` | Current mission status |

## Error Handling

### HTTP Status Codes

| Code | Description | Action Required |
|------|-------------|-----------------|
| `200` | Success | Operation completed successfully |
| `400` | Bad Request | Check request format and parameters |
| `401` | Unauthorized | Authentication required |
| `404` | Not Found | Verify endpoint URL |
| `422` | Unprocessable Entity | Invalid parameter values |
| `500` | Internal Server Error | Check server logs |
| `503` | Service Unavailable | Robot not ready or disconnected |

### WebSocket Error Messages

```json
{
  "type": "error",
  "code": "INVALID_COMMAND",
  "message": "Unknown command received",
  "timestamp": 1640995200.123
}
```

### ROS 2 Error Topics

Subscribe to `/diagnostics` for system health monitoring:
```bash
ros2 topic echo /diagnostics
```

## Security Considerations

### Authentication

#### HTTP API Security
```bash
# Basic authentication (if enabled)
curl -X POST http://localhost:5678/webhook/robot-control \
  -H "Content-Type: application/json" \
  -u "username:password" \
  -d '{"command": "forward"}'
```

#### WebSocket Security
```javascript
// Secure WebSocket connection
const ws = new WebSocket('wss://localhost:8765');

// Send authentication token
ws.send(JSON.stringify({
  type: 'auth',
  token: 'your_auth_token'
}));
```

### Rate Limiting

- **Movement Commands**: Maximum 10 commands per second
- **Emergency Stop**: Immediate response, no rate limiting
- **Status Requests**: Maximum 5 requests per second
- **Large Data Transfers**: Automatic throttling for sensor streams

### Input Validation

All API endpoints validate:
- Parameter types and ranges
- Message format compliance
- Security constraints
- Business logic rules

## Performance Characteristics

### API Response Times

| Operation | Typical Response | Maximum Response |
|-----------|------------------|------------------|
| Movement Command | < 10ms | < 50ms |
| Sensor Data Query | < 5ms | < 20ms |
| Emergency Stop | < 5ms | < 10ms |
| Status Check | < 2ms | < 10ms |

### Data Throughput

| Data Type | Update Rate | Data Size |
|-----------|-------------|-----------|
| LiDAR Scan | 30 Hz | ~2.5 KB |
| IMU Data | 50 Hz | ~200 bytes |
| Camera Stream | 30 Hz | ~500 KB |
| Odometry | 50 Hz | ~150 bytes |

## Integration Examples

### Python Integration

```python
#!/usr/bin/env python3
import requests
import json
import time

class RobotAPI:
    def __init__(self, base_url='http://localhost:5678'):
        self.base_url = base_url

    def move_forward(self, speed=0.5):
        response = requests.post(
            f'{self.base_url}/webhook/robot-control',
            json={'command': 'forward', 'speed': speed}
        )
        return response.json()

    def emergency_stop(self):
        response = requests.post(
            f'{self.base_url}/webhook/emergency-stop',
            json={'emergency': True}
        )
        return response.json()

    def get_status(self):
        response = requests.get(f'{self.base_url}/webhook/robot/status')
        return response.json()
```

### C++ Integration

```cpp
#include <curl/curl.h>
#include <nlohmann/json.hpp>

class RobotClient {
private:
    CURL* curl;
    std::string base_url;

public:
    RobotClient(std::string url) : base_url(url) {
        curl = curl_easy_init();
    }

    ~RobotClient() {
        curl_easy_cleanup(curl);
    }

    std::string sendCommand(const std::string& command, double speed = 0.5) {
        nlohmann::json payload = {
            {"command", command},
            {"speed", speed}
        };

        std::string response;
        // CURL implementation for HTTP POST
        return response;
    }
};
```

### LabVIEW Integration

```labview
# LabVIEW HTTP Request VI:
HTTP POST Request:
├── URL: http://localhost:5678/webhook/robot-control
├── Method: POST
├── Headers: Content-Type: application/json
├── Body: {"command": "forward", "speed": 0.5}
└── Response: Parse JSON result
```

## Monitoring and Debugging

### API Health Checks

```bash
# Check robot connectivity
curl -I http://localhost:5678/webhook/robot/status

# Monitor WebSocket connection
websocat ws://localhost:8765

# Check ROS 2 topic status
ros2 topic list
ros2 node list
```

### Log Files

- **n8n Logs**: `/n8n_data/n8nEventLog.log`
- **ROS 2 Logs**: Container logs via `docker logs ros2_sim_container`
- **System Logs**: Standard Linux system logs

### Debug Mode

Enable debug logging:
```bash
# Set environment variable for debug mode
export DEBUG=1

# Restart services to apply debug settings
docker compose restart
```

## Best Practices

### Error Handling

```javascript
// Robust error handling for API calls
async function safeRobotCommand(command, params) {
  try {
    const response = await fetch('/webhook/robot-control', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ command, ...params })
    });

    if (!response.ok) {
      throw new Error(`HTTP ${response.status}: ${response.statusText}`);
    }

    return await response.json();
  } catch (error) {
    console.error('Robot command failed:', error);
    // Implement retry logic or fallback behavior
  }
}
```

### Connection Management

```javascript
// WebSocket connection management
class RobotWebSocketManager {
  constructor(url) {
    this.url = url;
    this.ws = null;
    this.reconnectAttempts = 0;
    this.maxReconnectAttempts = 5;
  }

  connect() {
    this.ws = new WebSocket(this.url);

    this.ws.onopen = () => {
      this.reconnectAttempts = 0;
      console.log('Connected to robot');
    };

    this.ws.onclose = () => {
      if (this.reconnectAttempts < this.maxReconnectAttempts) {
        setTimeout(() => this.connect(), 1000 * this.reconnectAttempts);
        this.reconnectAttempts++;
      }
    };

    this.ws.onerror = (error) => {
      console.error('WebSocket error:', error);
    };
  }

  send(data) {
    if (this.ws && this.ws.readyState === WebSocket.OPEN) {
      this.ws.send(JSON.stringify(data));
    }
  }
}
```

## Support

For API integration questions:
- Review the specific API documentation sections
- Check the [Troubleshooting Guide](../troubleshooting/)
- Test with the provided examples first
- Open an issue on [GitHub](https://github.com/1999AZZAR/Autonomous_Mobile_Manipulator/issues) for API-related bugs

---

*This API documentation provides comprehensive information for integrating external systems with the Autonomous Mobile Manipulator robot through multiple communication interfaces.*
