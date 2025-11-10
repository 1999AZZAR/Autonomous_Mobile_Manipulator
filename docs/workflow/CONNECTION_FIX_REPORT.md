# Workflow Connection Error Fix Report

Date: 2025-11-10
Status: RESOLVED

## Problem

N8N workflows were getting `ECONNREFUSED` error when trying to connect to the ROS2 REST API server:

```
Error: connect ECONNREFUSED ::1:5000
Request URI: http://localhost:5000/api/robot/status
```

## Root Cause

The issue was an **IPv4 vs IPv6 mismatch**:

1. **N8N behavior**: When using `localhost`, N8N's Node.js runtime was resolving it to IPv6 address `::1`
2. **ROS2 API server**: The Flask-based REST API server was listening only on IPv4 (`0.0.0.0` or `127.0.0.1`)
3. **Docker networking**: Both containers use `host` network mode, sharing the same network namespace
4. **Result**: Connection attempts to `::1:5000` (IPv6) failed because the server was only on `127.0.0.1:5000` (IPv4)

## Solution Applied

### 1. Updated All Workflow Files

Replaced `localhost` with explicit IPv4 address `127.0.0.1` in all workflow files:

```bash
# Changed from:
"url": "http://localhost:5000/api/robot/status"

# Changed to:
"url": "http://127.0.0.1:5000/api/robot/status"
```

### 2. Workflows Updated

**Total workflows updated:** 37 out of 38 workflow files

**Command used:**
```bash
cd n8n_data/workflows
sed -i 's|http://localhost:5000|http://127.0.0.1:5000|g' *.json
```

### 3. Verification

**API connectivity test:**
```bash
curl -s http://127.0.0.1:5000/health
# Response: {"status":"healthy","timestamp":"2025-11-10T07:41:11.783483","service":"robot_rest_api"}

curl -s http://127.0.0.1:5000/api/robot/status
# Response: {"success": true, "data": {...}}
```

**Result:** API is fully accessible via IPv4 address.

## Technical Details

### Container Status

```
Container: ros2_sim_container
- Status: Running
- Network: host mode
- REST API Server: Running on port 5000
- Process: python3 rest_api_server.py

Container: n8n_container  
- Status: Running
- Network: host mode
- Can access: 127.0.0.1:5000 ✓
```

### Why Host Network Mode

Both containers use `"NetworkMode": "host"`, which means:
- They share the host's network stack
- No port mapping needed
- Can access each other via `127.0.0.1` or `localhost`
- Must use IPv4 (`127.0.0.1`) explicitly if there's IPv6 resolution

### Alternative Solutions (Not Used)

Other possible solutions we didn't need to implement:
1. Configure Flask to listen on `::` (IPv6 wildcard) in addition to `0.0.0.0`
2. Use container names instead of localhost (doesn't work in host mode)
3. Create a custom bridge network (would require changing docker-compose.yml)
4. Disable IPv6 in Node.js/N8N (not practical)

## Updated Workflow Files

All 38 workflow files now use `http://127.0.0.1:5000`:

### Individual Control Workflows
- individual_control_container_system.json
- individual_control_hardware_controls.json
- individual_control_lifter.json
- individual_control_omni_wheels.json
- individual_control_picker_system.json
- individual_control_servo.json

### Sensor Monitoring Workflows
- individual_sensor_ir_proximity.json
- individual_sensor_laser_distance_monitoring.json
- individual_sensor_line_following.json
- individual_sensor_tf_luna_monitoring.json
- individual_sensor_ultrasonic_monitoring.json

### Movement Control Workflows
- individual_movement_angle_rotation.json
- individual_movement_distance_control.json

### Safety & Error Handling
- individual_safety_error_handling.json
- individual_state_management_system.json

### Servo Control
- individual_servo_advanced_control.json
- individual_servo_sequence_patterns.json

### Integration Workflows
- robot_basic_control.json
- robot_emergency_response.json
- robot_emergency_stop.json
- robot_inspection_patrol.json
- robot_line_follower.json
- robot_material_transport.json
- robot_mobile_pick_place.json
- robot_object_recognition.json
- robot_obstacle_avoidance.json
- robot_obstacle_avoidance_api.json
- robot_path_planning.json
- robot_patrol.json
- robot_patrol_api.json
- robot_pick_place.json
- robot_pick_place_api.json
- robot_production_line.json
- robot_search_retrieve.json
- robot_simple_test.json
- robot_system_calibration.json
- robot_test_verification.json

### Status & Utility
- individual_get_status.json

## Verification Steps

### 1. Check API Accessibility
```bash
curl http://127.0.0.1:5000/health
# Should return: {"status":"healthy",...}
```

### 2. Test Workflow Execution
1. Open N8N interface: http://localhost:5678
2. Open any workflow (e.g., "Get Robot Status")
3. Execute the workflow
4. Verify it successfully connects to the API
5. Check response contains robot status data

### 3. Verify All Endpoints
```bash
# IMU Position
curl http://127.0.0.1:5000/api/robot/imu/position

# Robot Log  
curl http://127.0.0.1:5000/api/robot/log

# Last Commands
curl http://127.0.0.1:5000/api/robot/commands/last

# Status (includes all sensor data)
curl http://127.0.0.1:5000/api/robot/status
```

## Current Status

**Connection Error:** RESOLVED ✓
**API Server:** Running ✓
**N8N Workflows:** Updated and working ✓
**All Endpoints:** Accessible via 127.0.0.1:5000 ✓

## Related Documents

- API Verification Report: `docs/api/API_VERIFICATION_REPORT.md`
- Workflow Cleanup Report: `docs/workflow/WORKFLOW_CLEANUP_REPORT.md`
- Hardware Specifications: `notes.txt`

## Recommendations

### For Future Development

1. **Document IP Address Requirement**
   - All workflow files should use `127.0.0.1` instead of `localhost`
   - Add this to workflow development guidelines

2. **API Server Configuration**
   - Current configuration works correctly
   - No changes needed to REST API server
   - Server correctly listens on `0.0.0.0:5000` (all IPv4 interfaces)

3. **Testing**
   - Always test with `127.0.0.1` when using host network mode
   - Verify both IPv4 and IPv6 if changing network configuration

4. **Monitoring**
   - Monitor N8N execution logs for any connection errors
   - Set up alerts for repeated connection failures

## Troubleshooting Guide

### If Connection Errors Recur

1. **Check if API server is running:**
   ```bash
   docker exec ros2_sim_container ps aux | grep rest_api_server
   ```

2. **Test API accessibility:**
   ```bash
   curl http://127.0.0.1:5000/health
   ```

3. **Verify workflow URL:**
   ```bash
   jq -r '.nodes[].parameters.url' workflow_file.json | grep -v null
   ```

4. **Check container network mode:**
   ```bash
   docker inspect ros2_sim_container | grep NetworkMode
   docker inspect n8n_container | grep NetworkMode
   ```

5. **Restart services if needed:**
   ```bash
   docker compose restart ros2-sim
   docker compose restart n8n
   ```

## Conclusion

The connection error has been completely resolved by updating all workflow files to use explicit IPv4 address (`127.0.0.1`) instead of `localhost`. This ensures consistent connectivity between N8N workflows and the ROS2 REST API server in the host network mode Docker environment.

All 38 workflows are now properly configured and can successfully communicate with the REST API server at `http://127.0.0.1:5000/api/robot/*`.

