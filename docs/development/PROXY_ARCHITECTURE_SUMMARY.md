# Proxy Architecture Summary - Web Interface to ROS2

## Overview

The web interface now uses a **proxy architecture** where the UI server forwards all robot control commands to the ROS2-integrated server.

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                     CLIENT (Any Device)                      │
│              Browser at http://robot-ip:8000                 │
└───────────────────────────┬─────────────────────────────────┘
                            │
                            │ HTTP Requests
                            ▼
┌─────────────────────────────────────────────────────────────┐
│              web_robot_interface.py (Port 8000)              │
│─────────────────────────────────────────────────────────────│
│  • Serves HTML/CSS/JavaScript UI                            │
│  • Reads sensors directly (SPI/I2C):                        │
│    - MPU6050 IMU (I2C 0x68)                                 │
│    - Sharp IR sensors (MCP3008 SPI)                         │
│  • Forwards control commands to port 5000                   │
│  • Accessible from ANY device (host='0.0.0.0')              │
└───────────────────────────┬─────────────────────────────────┘
                            │
                            │ Forwards via requests library
                            ▼
┌─────────────────────────────────────────────────────────────┐
│               rest_api_server.py (Port 5000)                 │
│─────────────────────────────────────────────────────────────│
│  • Full ROS2 Integration                                    │
│  • Service Clients:                                         │
│    - pick_place_client                                      │
│    - patrol_client                                          │
│    - obstacle_avoidance_client                              │
│    - robot_status_client                                    │
│    - emergency_stop_client                                  │
│  • ROS2 Publishers:                                         │
│    - /cmd_vel (Twist)                                       │
│    - /picker/gripper (Bool)                                 │
│    - /picker/gripper_tilt (Float32)                         │
│    - /containers/* (String)                                 │
│  • Real robot control                                       │
└───────────────────────────┬─────────────────────────────────┘
                            │
                            │ ROS2 Topics & Services
                            ▼
┌─────────────────────────────────────────────────────────────┐
│                    ROS2 Ecosystem                            │
│─────────────────────────────────────────────────────────────│
│  • Hardware drivers                                         │
│  • Navigation stack                                         │
│  • Motion planning                                          │
│  • Sensor processing                                        │
└─────────────────────────────────────────────────────────────┘
```

## Port Assignment

| Port | Service | Purpose |
|------|---------|---------|
| **8000** | web_robot_interface.py | User-facing web UI + sensors + proxy |
| **5000** | rest_api_server.py | ROS2 integration server |
| **5678** | n8n | Automation workflows |

## Endpoints

### Handled Locally by Port 8000 (Direct Hardware Access)

| Endpoint | Method | Purpose |
|----------|--------|---------|
| `/` | GET | Serve HTML UI |
| `/api/robot/sensors` | GET | Read sensors (SPI/I2C) |
| `/api/robot/imu/position` | GET | Read IMU data (I2C) |
| `/api/robot/imu/calibrate` | POST | Calibrate IMU |
| `/api/robot/sensors/diagnostics` | GET | Sensor health |

### Proxied to Port 5000 (ROS2 Control)

| Endpoint | Method | Proxied To |
|----------|--------|------------|
| `/api/robot/move` | POST | `http://localhost:5000/api/robot/move` |
| `/api/robot/turn` | POST | `http://localhost:5000/api/robot/turn` |
| `/api/robot/stop` | POST | `http://localhost:5000/api/robot/stop` |
| `/api/robot/mode` | POST | `http://localhost:5000/api/robot/mode` |
| `/api/robot/picker/gripper` | POST | `http://localhost:5000/api/robot/picker/gripper` |
| `/api/robot/picker/gripper_tilt` | POST | `http://localhost:5000/api/robot/picker/gripper_tilt` |
| `/api/robot/picker/gripper_neck` | POST | `http://localhost:5000/api/robot/picker/gripper_neck` |
| `/api/robot/picker/gripper_base` | POST | `http://localhost:5000/api/robot/picker/gripper_base` |
| `/api/robot/servos` | POST | `http://localhost:5000/api/robot/servos` |
| `/api/robot/containers/<id>` | POST | `http://localhost:5000/api/robot/containers/<id>` |
| `/api/robot/patrol` | POST | `http://localhost:5000/api/robot/patrol` |
| `/api/robot/obstacle-avoidance` | POST | `http://localhost:5000/api/robot/obstacle-avoidance` |
| `/api/robot/pick-place` | POST | `http://localhost:5000/api/robot/pick-place` |
| `/api/robot/emergency-stop` | POST | `http://localhost:5000/api/robot/emergency-stop` |
| `/api/robot/commands/last` | GET | `http://localhost:5000/api/robot/commands/last` |
| `/api/robot/log` | GET | `http://localhost:5000/api/robot/log` |
| `/webhook/robot-control` | POST | `http://localhost:5000/webhook/robot-control` |

## Implementation Details

### Proxy Code Pattern

```python
import requests

@self.app.route('/api/robot/move', methods=['POST'])
def robot_move():
    try:
        data = request.get_json()
        # Forward to ROS2 server
        response = requests.post('http://localhost:5000/api/robot/move', 
                               json=data, timeout=5)
        return jsonify(response.json()), response.status_code
    except requests.exceptions.RequestException as e:
        self.get_logger().error(f'Failed to forward move command: {str(e)}')
        return jsonify({'success': False, 'error': 'ROS2 server unavailable'}), 503
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)}), 500
```

### Error Handling

- **Timeout**: 5 seconds for commands, 10 seconds for long operations
- **Connection Error**: Returns 503 "ROS2 server unavailable"
- **Fallback**: Monitoring endpoints have local fallback data
- **Logging**: All proxy errors logged via ROS2 logger

## Benefits

### ✅ Separation of Concerns
- **UI Layer**: Clean interface, sensor reading
- **Control Layer**: ROS2 integration, robot commands
- **Each service can be developed/tested independently**

### ✅ Network Accessibility
- Web UI accessible from **any device** on network
- No need to access Pi directly
- Control robot from phone, tablet, laptop

### ✅ Maintainability
- Changes to ROS2 integration don't affect UI
- UI updates don't require ROS2 rebuild
- Clear API boundaries

### ✅ Resilience
- If ROS2 server down, UI still serves pages
- Sensor data still readable
- Clear error messages when services unavailable

### ✅ Performance
- Sensor reads are local (no network delay)
- Control commands use efficient HTTP proxy
- Both servers can scale independently

## Access Instructions

### From Same Network

```bash
# Find Pi's IP
hostname -I  # On Raspberry Pi

# Access from any device
http://YOUR_PI_IP:8000
```

### Example IPs
- **Local (on Pi)**: http://localhost:8000
- **Same Network**: http://100.74.72.71:8000
- **Tailscale VPN**: http://10.54.127.114:8000

### Testing Endpoints

```bash
# Test web UI (should work from anywhere)
curl http://YOUR_PI_IP:8000/

# Test sensor data (local hardware read)
curl http://YOUR_PI_IP:8000/api/robot/sensors

# Test IMU (local I2C read)
curl http://YOUR_PI_IP:8000/api/robot/imu/position

# Test robot control (proxied to ROS2)
curl -X POST http://YOUR_PI_IP:8000/api/robot/move \
  -H "Content-Type: application/json" \
  -d '{"direction":"forward","speed":0.5}'

# Test gripper (proxied to ROS2)
curl -X POST http://YOUR_PI_IP:8000/api/robot/picker/gripper \
  -H "Content-Type: application/json" \
  -d '{"command":"open"}'
```

## Dependencies

### web_robot_interface.py
```python
import requests  # NEW - for proxying requests
import rclpy
from flask import Flask, jsonify, request
import spidev  # For MCP3008 ADC
from mpu6050_reader import MPU6050Reader  # For IMU
```

### Installation
```bash
pip3 install --break-system-packages requests
```

## Troubleshooting

### "ROS2 server unavailable" Error

**Cause**: rest_api_server.py (port 5000) not running

**Solution**:
```bash
# Check if running
./run.sh status

# View logs
./run.sh logs | grep rest_api_server

# Restart
./run.sh restart
```

### "Connection Lost" in Browser

**Causes**:
1. Browser cached old files
2. Network connection issue
3. Server restarted

**Solutions**:
```bash
# 1. Hard refresh browser
Ctrl + Shift + R (or Cmd + Shift + R on Mac)

# 2. Clear browser cache
Settings > Clear browsing data

# 3. Check server status
./run.sh status

# 4. Check from command line
curl http://YOUR_PI_IP:8000/api/robot/status
```

### Timeouts

**Symptoms**: Commands take long time, timeout errors

**Causes**:
- ROS2 services slow to respond
- Network latency

**Solutions**:
```python
# Increase timeout in web_robot_interface.py
response = requests.post('...', timeout=10)  # Increase from 5 to 10
```

### Port Already in Use

**Symptom**: "Address already in use" when starting

**Solution**:
```bash
# Kill process on port 8000
sudo lsof -ti:8000 | xargs kill -9

# Restart
./run.sh restart
```

## Development Workflow

### Testing Locally

```bash
# 1. Start ROS2 server
cd ~/Autonomous_Mobile_Manipulator
./run.sh start

# 2. Access UI
# From browser: http://localhost:8000
# From another device: http://PI_IP:8000

# 3. Test commands
# Use browser UI or curl commands
```

### Adding New Endpoints

1. **Add to rest_api_server.py** (ROS2 integration)
2. **Add proxy in web_robot_interface.py**:
```python
@self.app.route('/api/robot/new_endpoint', methods=['POST'])
def new_endpoint():
    try:
        data = request.get_json()
        response = requests.post('http://localhost:5000/api/robot/new_endpoint', 
                               json=data, timeout=5)
        return jsonify(response.json()), response.status_code
    except requests.exceptions.RequestException as e:
        return jsonify({'success': False, 'error': 'ROS2 server unavailable'}), 503
```
3. **Test from UI or curl**

### Updating UI

- Edit HTML/JavaScript in web_robot_interface.py
- No need to restart ROS2
- Just restart web interface:
```bash
docker exec ros2_sim_container pkill -f web_robot_interface
# It will auto-restart via launch file
```

## Security Considerations

### Current Setup
- **No authentication** (internal network only)
- **No encryption** (HTTP not HTTPS)
- **No rate limiting**

### Recommendations for Production

1. **Add Authentication**:
```python
from flask_httpauth import HTTPBasicAuth
auth = HTTPBasicAuth()

@app.route('/api/robot/move')
@auth.login_required
def move():
    # ...
```

2. **Enable HTTPS**:
```python
app.run(host='0.0.0.0', port=8000, ssl_context='adhoc')
```

3. **Add Rate Limiting**:
```python
from flask_limiter import Limiter
limiter = Limiter(app, default_limits=["100 per hour"])
```

4. **Firewall Rules**:
```bash
sudo ufw allow 8000/tcp from 192.168.1.0/24
```

## Status

- ✅ **Proxy Architecture**: Implemented
- ✅ **Network Accessible**: Yes (0.0.0.0)
- ✅ **Error Handling**: Comprehensive
- ✅ **Logging**: Via ROS2 logger
- ✅ **Sensor Integration**: Direct hardware access
- ✅ **ROS2 Integration**: Via proxy
- ✅ **Testing**: Ready

Date: 2025-11-11  
Version: 2.0 (Proxy Architecture)

