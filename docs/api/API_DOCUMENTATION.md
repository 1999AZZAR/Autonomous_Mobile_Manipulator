# Robot Web Interface API Documentation

## Base URL

```
http://<robot-ip>:8000
```

Default port: 8000

## API Endpoints

### 1. Health Check

Check if the web interface is running.

**Endpoint:** `GET /health`

**Response:**
```json
{
  "status": "healthy",
  "service": "robot_web_interface",
  "timestamp": 1699564800.123
}
```

**Status Codes:**
- 200: Service is healthy

---

### 2. Get Sensor Data

Retrieve current readings from all sensors.

**Endpoint:** `GET /api/robot/sensors`

**Response:**
```json
{
  "success": true,
  "data": {
    "laser_sensors": {
      "left_front": 450,
      "left_back": 520,
      "right_front": 380,
      "right_back": 610,
      "back_left": 725,
      "back_right": 690
    },
    "ultrasonic_sensors": {
      "front_left": null,
      "front_right": null
    },
    "tf_luna": {
      "distance": null,
      "strength": null
    },
    "line_sensors": {
      "left": false,
      "center": false,
      "right": false
    },
    "container_sensors": {
      "left_front": false,
      "left_back": false,
      "right_front": false,
      "right_back": false
    },
    "imu": {
      "yaw": 0.0,
      "pitch": 0.0,
      "roll": 0.0
    }
  },
  "timestamp": 1699564800.123
}
```

**Notes:**
- `laser_sensors`: Distance in millimeters (200-1500 mm range)
- `null` values indicate sensor not implemented or offline
- Update rate: ~1 Hz

**Status Codes:**
- 200: Success
- 500: Sensor read error

---

### 3. Get Sensor Diagnostics

Retrieve sensor health and diagnostic information.

**Endpoint:** `GET /api/robot/sensors/diagnostics`

**Response:**
```json
{
  "success": true,
  "data": {
    "simulation_mode": false,
    "spi_initialized": true,
    "sensor_health": {
      "left_front": {
        "status": "healthy",
        "last_read": "2025-11-11T02:15:30.123456",
        "error_count": 0
      },
      "left_back": {
        "status": "healthy",
        "last_read": "2025-11-11T02:15:30.456789",
        "error_count": 0
      },
      "right_front": {
        "status": "error",
        "last_read": "2025-11-11T02:14:15.789012",
        "error_count": 3
      },
      "right_back": {
        "status": "healthy",
        "last_read": "2025-11-11T02:15:30.234567",
        "error_count": 0
      },
      "back_left": {
        "status": "no_data",
        "last_read": null,
        "error_count": 5
      },
      "back_right": {
        "status": "healthy",
        "last_read": "2025-11-11T02:15:30.345678",
        "error_count": 0
      }
    },
    "adc_config": {
      "vref": 3.3,
      "resolution": 1024,
      "spi_speed": 1350000
    }
  },
  "timestamp": 1699564800.123
}
```

**Sensor Status Values:**
- `healthy`: Sensor operating normally
- `error`: Sensor read error occurred
- `no_data`: Sensor returning no valid data
- `unknown`: Sensor not yet read

**Status Codes:**
- 200: Success
- 500: Error retrieving diagnostics

---

### 4. Get System Status

Retrieve overall robot system status.

**Endpoint:** `GET /api/robot/status`

**Response:**
```json
{
  "success": true,
  "data": {
    "mode": "MANUAL",
    "emergency_stop": false,
    "battery_voltage": 24.0,
    "system_status": "operational",
    "simulation_mode": false,
    "spi_initialized": true
  },
  "timestamp": 1699564800.123
}
```

**Status Codes:**
- 200: Success

---

## Data Types

### Distance Sensor Data

Sharp GP2Y0A02YK0F IR Distance Sensors:
- Type: `integer` or `null`
- Unit: millimeters (mm)
- Range: 200-1500 mm
- Typical accuracy: ±10 mm
- Update rate: ~1 Hz

### Sensor Health Status

```typescript
{
  status: "healthy" | "error" | "no_data" | "unknown",
  last_read: string | null,  // ISO 8601 timestamp
  error_count: number
}
```

### System Modes

- `MANUAL`: Manual control via web interface
- `AUTONOMOUS`: Autonomous operation
- `MAINTENANCE`: Maintenance mode

## Error Responses

All error responses follow this format:

```json
{
  "success": false,
  "error": "Error description",
  "timestamp": 1699564800.123
}
```

Common HTTP status codes:
- 400: Bad Request
- 404: Not Found
- 500: Internal Server Error

## Simulation Mode

When hardware is not available, the system operates in simulation mode:

- `simulation_mode`: `true` in status response
- `spi_initialized`: `false`
- Sensor data is simulated with sinusoidal patterns
- All API endpoints remain functional
- Useful for development and testing

## Rate Limiting

No rate limiting currently implemented. Recommended client-side polling:
- Sensor data: 1 Hz
- Diagnostics: 0.1 Hz (every 10 seconds)
- Status: 0.5 Hz

## WebSocket Support

Not currently implemented. All communication via HTTP REST API.

Future consideration: WebSocket endpoint for real-time sensor streaming.

## Authentication

Not currently implemented. All endpoints are publicly accessible.

For production deployment, consider adding:
- API key authentication
- HTTPS/TLS encryption
- IP whitelisting

## Example Usage

### Python

```python
import requests
import time

BASE_URL = "http://192.168.1.100:8000"

# Get sensor data
response = requests.get(f"{BASE_URL}/api/robot/sensors")
data = response.json()

if data['success']:
    sensors = data['data']['laser_sensors']
    print(f"Left Front: {sensors['left_front']} mm")
    print(f"Right Front: {sensors['right_front']} mm")

# Monitor sensors continuously
while True:
    response = requests.get(f"{BASE_URL}/api/robot/sensors")
    data = response.json()
    
    if data['success']:
        sensors = data['data']['laser_sensors']
        print(f"Sensors: {sensors}")
    
    time.sleep(1)
```

### JavaScript

```javascript
const BASE_URL = "http://192.168.1.100:8000";

// Get sensor data
async function getSensorData() {
  const response = await fetch(`${BASE_URL}/api/robot/sensors`);
  const data = await response.json();
  
  if (data.success) {
    console.log("Laser Sensors:", data.data.laser_sensors);
  }
}

// Monitor sensors continuously
setInterval(async () => {
  const response = await fetch(`${BASE_URL}/api/robot/sensors`);
  const data = await response.json();
  
  if (data.success) {
    console.log("Left Front:", data.data.laser_sensors.left_front, "mm");
  }
}, 1000);
```

### cURL

```bash
# Get sensor data
curl http://192.168.1.100:8000/api/robot/sensors | jq

# Get diagnostics
curl http://192.168.1.100:8000/api/robot/sensors/diagnostics | jq

# Get system status
curl http://192.168.1.100:8000/api/robot/status | jq

# Monitor sensors (continuous)
watch -n 1 'curl -s http://192.168.1.100:8000/api/robot/sensors | jq ".data.laser_sensors"'
```

## Integration Examples

### ROS2 Node

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
import requests

class SensorBridge(Node):
    def __init__(self):
        super().__init__('sensor_bridge')
        self.api_url = "http://localhost:8000/api/robot/sensors"
        
        # Create publishers for each sensor
        self.publishers = {}
        for sensor in ['left_front', 'left_back', 'right_front', 
                      'right_back', 'back_left', 'back_right']:
            self.publishers[sensor] = self.create_publisher(
                Range, f'/sensors/{sensor}', 10
            )
        
        # Poll API at 1 Hz
        self.timer = self.create_timer(1.0, self.update_sensors)
    
    def update_sensors(self):
        try:
            response = requests.get(self.api_url)
            data = response.json()
            
            if data['success']:
                for name, distance in data['data']['laser_sensors'].items():
                    if distance is not None:
                        msg = Range()
                        msg.header.stamp = self.get_clock().now().to_msg()
                        msg.header.frame_id = f"sensor_{name}"
                        msg.radiation_type = Range.INFRARED
                        msg.min_range = 0.2  # 200 mm
                        msg.max_range = 1.5  # 1500 mm
                        msg.range = distance / 1000.0  # Convert to meters
                        
                        self.publishers[name].publish(msg)
        except Exception as e:
            self.get_logger().error(f'Failed to update sensors: {e}')

def main():
    rclpy.init()
    node = SensorBridge()
    rclpy.spin(node)
    rclpy.shutdown()
```

### n8n Workflow

```json
{
  "nodes": [
    {
      "name": "Poll Sensors",
      "type": "n8n-nodes-base.httpRequest",
      "parameters": {
        "method": "GET",
        "url": "http://192.168.1.100:8000/api/robot/sensors",
        "options": {}
      }
    },
    {
      "name": "Check Distance",
      "type": "n8n-nodes-base.if",
      "parameters": {
        "conditions": {
          "number": [
            {
              "value1": "={{$json.data.laser_sensors.left_front}}",
              "operation": "smaller",
              "value2": 300
            }
          ]
        }
      }
    },
    {
      "name": "Alert",
      "type": "n8n-nodes-base.sendEmail",
      "parameters": {
        "subject": "Robot Obstacle Alert",
        "text": "Object detected closer than 300mm"
      }
    }
  ]
}
```

## Changelog

### Version 1.0 (2025-11-11)
- Initial API implementation
- Added sensor data endpoint
- Added diagnostics endpoint
- Added system status endpoint
- Implemented simulation mode
- Added sensor health tracking

## Future Enhancements

Planned features:
- WebSocket support for real-time streaming
- Authentication and authorization
- Historical data logging
- Sensor calibration API
- Configuration management API
- Batch sensor requests
- Event streaming for sensor alerts

