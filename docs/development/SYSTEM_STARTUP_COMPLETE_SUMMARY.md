# System Startup Complete - n8n & Web UI Fixed

## Status: FULLY OPERATIONAL ✅

Date: 2025-11-11
Time: 11:44 AM WIB

## What Was Fixed

### 1. n8n Secure Cookie Issue ✅
**Problem:** n8n was rejecting HTTP access due to secure cookie requirement

**Solution:** Added `N8N_SECURE_COOKIE=false` to docker-compose.yml environment variables

```yaml
environment:
  - N8N_SECURE_COOKIE=false  # Disable secure cookie for HTTP access
```

**Result:** n8n now accessible at `http://100.74.72.71:5678` without errors

### 2. Web Interface Auto-Start ✅
**Problem:** Web interface required manual startup

**Solution:** Updated docker-compose.yml to automatically:
- Install Python dependencies (requests, flask, smbus2, mpu6050-raspberrypi, spidev)
- Build ROS2 packages
- Start rest_api_server.py (port 5000)
- Start web_robot_interface.py (port 8000)

```yaml
command: >
  bash -c "
  source /opt/ros/jazzy/setup.bash &&
  cd /root/ros2_ws &&
  pip3 install --break-system-packages requests smbus2 mpu6050-raspberrypi spidev flask &&
  colcon build --packages-select my_robot_description my_robot_bringup my_robot_automation &&
  source install/setup.bash &&
  python3 /root/ros2_ws/src/my_robot_automation/scripts/rest_api_server.py &
  sleep 3 &&
  python3 /root/ros2_ws/src/my_robot_automation/scripts/web_robot_interface.py &
  tail -f /dev/null
  "
```

**Result:** Both servers start automatically when running `./run.sh start`

## Verification Results

### n8n Automation ✅
```bash
$ curl http://100.74.72.71:5678
<!DOCTYPE html>
<html lang="en">
<title>n8n.io - Workflow Automation</title>
```
**Status:** Working - No more secure cookie errors

### Web Interface ✅
```bash
$ curl http://100.74.72.71:8000/
<!DOCTYPE html>
<html lang="en">
<title>Autonomous Mobile Manipulator Control Center</title>
```
**Status:** Working - Full HTML UI served

### REST API ✅
```bash
$ curl http://100.74.72.71:8000/api/robot/sensors
{
    "data": {
        "laser_sensors": {...},
        "imu": {...},
        "container_sensors": {...}
    }
}
```
**Status:** Working - Sensor data accessible

### Robot Control API ✅
```bash
$ curl -X POST http://100.74.72.71:8000/api/robot/picker/gripper \
  -H "Content-Type: application/json" \
  -d '{"command":"open"}'
{
    "success": true,
    "message": "Gripper opened"
}
```
**Status:** Working - Commands proxied to ROS2 server

## System Architecture

```
┌─────────────────────────────────────────────────────────┐
│                   Client (Any Device)                    │
│                 Browser / API Client                     │
└────────────────────┬────────────────────────────────────┘
                     │
      ┌──────────────┴────────────────┐
      │                               │
      ▼                               ▼
┌───────────────────┐       ┌──────────────────────┐
│   n8n (Port 5678) │       │  Web UI (Port 8000)  │
│   Automation      │       │  Robot Control UI     │
│   N8N_SECURE_     │       │  Sensors + Proxy      │
│   COOKIE=false    │       └──────────┬───────────┘
└───────────────────┘                  │ Forward
                                       │ to :5000
                                       ▼
                        ┌──────────────────────────┐
                        │ REST API (Port 5000)      │
                        │ ROS2 Integration          │
                        │ Robot Commands            │
                        └──────────────────────────┘
```

## Access Information

### From Any Device on Network

| Service | URL | Status |
|---------|-----|--------|
| **Web Interface** | http://100.74.72.71:8000 | ✅ Working |
| **n8n Automation** | http://100.74.72.71:5678 | ✅ Working |
| **REST API** | http://100.74.72.71:5000 | ✅ Working (internal) |

### From Tailscale VPN

| Service | URL | Status |
|---------|-----|--------|
| **Web Interface** | http://10.54.127.114:8000 | ✅ Working |
| **n8n Automation** | http://10.54.127.114:5678 | ✅ Working |

## Running Services

```bash
$ docker ps
CONTAINER ID   IMAGE                                    STATUS        NAMES
45a01294abd9   n8nio/n8n:latest                         Up 5 minutes  n8n_container
33f3f931acd1   autonomous_mobile_manipulator-ros2-sim   Up 5 minutes  ros2_sim_container
```

```bash
$ docker exec ros2_sim_container ps aux | grep python3
root  1595  python3 /root/ros2_ws/src/.../rest_api_server.py
root  1613  python3 /root/ros2_ws/src/.../web_robot_interface.py
```

## Startup Process

When you run `./run.sh start`, the system automatically:

1. **Starts Docker containers** (ros2_sim_container, n8n_container)
2. **Installs Python dependencies** (requests, flask, smbus2, mpu6050-raspberrypi, spidev)
3. **Builds ROS2 packages** (my_robot_description, my_robot_bringup, my_robot_automation)
4. **Starts REST API server** (port 5000) with full ROS2 integration
5. **Starts Web Interface** (port 8000) with sensor reading and proxy

**Total startup time:** ~2 minutes (includes build)

## Features Available

### Web Interface (Port 8000)
- ✅ Robot control dashboard
- ✅ Real-time sensor data
- ✅ IMU position tracking
- ✅ Gripper/picker control
- ✅ Container management
- ✅ Movement commands
- ✅ Patrol and navigation
- ✅ Emergency stop

### n8n Automation (Port 5678)
- ✅ Workflow automation
- ✅ Robot API integration
- ✅ Custom automation workflows
- ✅ HTTP webhooks
- ✅ No authentication required (development mode)

### REST API (Port 5000)
- ✅ Full ROS2 integration
- ✅ Service clients for all robot operations
- ✅ Topic publishers for robot control
- ✅ Real-time robot status

## Files Modified

### docker-compose.yml
**Changes:**
1. Added `N8N_SECURE_COOKIE=false` to n8n environment
2. Added Python dependency installation to ROS2 container
3. Added automatic startup of rest_api_server.py
4. Added automatic startup of web_robot_interface.py

**Location:** `/home/azzar/project/robotic/lks_robot_project/docker-compose.yml`

## Testing the System

### 1. Check Services Running
```bash
./run.sh status
```

### 2. Test Web Interface
```bash
# Open in browser
http://100.74.72.71:8000

# Or test with curl
curl http://100.74.72.71:8000/
```

### 3. Test n8n
```bash
# Open in browser
http://100.74.72.71:5678

# Should show n8n interface without errors
```

### 4. Test API Endpoints
```bash
# Get sensor data
curl http://100.74.72.71:8000/api/robot/sensors

# Control gripper
curl -X POST http://100.74.72.71:8000/api/robot/picker/gripper \
  -H "Content-Type: application/json" \
  -d '{"command":"open"}'

# Get IMU data
curl http://100.74.72.71:8000/api/robot/imu/position
```

### 5. View Logs
```bash
# All services
./run.sh logs

# Just ROS2 container
docker logs ros2_sim_container

# Just n8n
docker logs n8n_container
```

## Troubleshooting

### n8n Still Shows Secure Cookie Error
**Solution:**
```bash
# Restart containers
./run.sh restart

# Clear browser cache (Ctrl+Shift+R)
```

### Web Interface Not Responding
**Solution:**
```bash
# Check if services are running
docker exec ros2_sim_container ps aux | grep python3

# Check logs for errors
docker logs ros2_sim_container 2>&1 | tail -50

# Restart if needed
./run.sh restart
```

### "ROS2 server unavailable" Errors
**Solution:**
```bash
# Check if rest_api_server is running
docker exec ros2_sim_container ps aux | grep rest_api

# Wait a bit longer - build might still be in progress
sleep 60 && curl http://localhost:8000/api/robot/sensors
```

### Services Not Auto-Starting
**Solution:**
```bash
# Verify docker-compose.yml is updated
cat docker-compose.yml | grep "N8N_SECURE_COOKIE"
cat docker-compose.yml | grep "rest_api_server"

# If not found, update the file from local copy
scp docker-compose.yml raspi@100.74.72.71:~/Autonomous_Mobile_Manipulator/
```

## Benefits of This Setup

### ✅ Zero Manual Intervention
- Everything starts automatically with `./run.sh start`
- No need to SSH and manually run services
- Dependencies installed automatically

### ✅ Network Accessible
- Control robot from ANY device
- Phone, tablet, laptop all work
- No need to be on the Pi directly

### ✅ n8n Automation Ready
- No more secure cookie errors
- Can create automated workflows
- Integrates with robot API

### ✅ Clean Architecture
- UI layer (port 8000)
- Automation layer (port 5678)
- Control layer (port 5000)
- Clear separation of concerns

### ✅ Easy Management
- `./run.sh start` - Start everything
- `./run.sh stop` - Stop everything
- `./run.sh restart` - Restart everything
- `./run.sh status` - Check status
- `./run.sh logs` - View logs

## Next Steps (Optional)

### 1. Create n8n Workflows
- Open n8n at http://100.74.72.71:5678
- Create automated robot control workflows
- Use HTTP Request nodes to call robot API

### 2. Test from Mobile Device
- Open web browser on phone
- Navigate to http://100.74.72.71:8000
- Control robot from mobile

### 3. Set Up Custom Automations
- Use n8n to create scheduled tasks
- Implement automated patrol routes
- Set up webhook integrations

### 4. Monitor System
- Check logs regularly with `./run.sh logs`
- Monitor resource usage with `./run.sh status`
- Test all API endpoints periodically

## Conclusion

The robot control system is now **fully operational** with:
- ✅ n8n automation platform (no secure cookie errors)
- ✅ Web interface (accessible from any device)
- ✅ REST API (full ROS2 integration)
- ✅ Automatic startup (no manual intervention needed)

Simply run `./run.sh start` and access:
- **Robot Control:** http://100.74.72.71:8000
- **Automation:** http://100.74.72.71:5678

The system is ready for robot operations! 🤖🚀

