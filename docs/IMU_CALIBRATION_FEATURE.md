# IMU Calibration Feature

**Last Updated:** 2025-11-10  
**Status:** Implemented and Operational

## Overview

The IMU (Inertial Measurement Unit) calibration feature allows you to set a zero reference point for the MPU6050 sensor. This is essential for accurate orientation tracking, especially after robot movement or when starting a new session.

## Purpose

### Why IMU Calibration is Needed

1. **Drift Compensation:** IMUs accumulate small errors over time (drift)
2. **Initial Orientation:** Sets a known reference point at startup
3. **Coordinate System Alignment:** Aligns sensor frame with robot frame
4. **Improved Accuracy:** Reduces accumulated orientation errors

### When to Calibrate

- After robot startup
- When the robot is placed in a known orientation
- After detecting significant drift
- Before precision navigation tasks
- When orientation readings seem incorrect

## Implementation

### API Endpoint

**POST /api/robot/imu/calibrate**

**Request:**
```bash
curl -X POST http://127.0.0.1:5000/api/robot/imu/calibrate \
  -H "Content-Type: application/json"
```

**Response (Success):**
```json
{
  "success": true,
  "message": "IMU calibration completed",
  "data": {
    "calibrated": true,
    "zero_reference": {
      "roll": 0.0,
      "pitch": 0.0,
      "yaw": 0.0
    },
    "timestamp": 1699654321.123
  }
}
```

**Response (Error):**
```json
{
  "success": false,
  "error": "Error message here"
}
```

### Web Interface Control

**Location:** Sensors Tab → IMU (MPU6050) Section

**Button:**
- Label: "Calibrate IMU (Set Zero)"
- Icon: Crosshairs (target symbol)
- Color: Warning (orange) to indicate important action

**Status Display:**
- Shows calibration progress
- Displays success/failure messages
- Auto-clears after 3 seconds on success
- Color-coded feedback:
  - Orange: Calibrating in progress
  - Green: Success
  - Red: Failure

### User Experience Flow

1. **User clicks "Calibrate IMU (Set Zero)" button**
2. **Status shows:** "Calibrating IMU..." (orange)
3. **API call made** to `/api/robot/imu/calibrate`
4. **On Success:**
   - Status shows: "✓ IMU Calibrated - Zero reference set" (green)
   - Log entry added: "IMU calibration completed successfully"
   - Message auto-clears after 3 seconds
5. **On Failure:**
   - Status shows: "✗ Calibration failed: [error]" (red)
   - Log entry added with error details
   - Message persists until next action

## Technical Details

### Current Implementation

The current implementation is a **simulation/placeholder** that:
- Accepts calibration requests
- Returns success responses
- Logs calibration events
- Sets zero reference values

### Real Hardware Integration

For actual MPU6050 hardware, the implementation should:

1. **Read Current Sensor State:**
   ```python
   current_orientation = read_mpu6050_orientation()
   ```

2. **Store as Zero Reference:**
   ```python
   zero_reference = {
       'roll': current_orientation.roll,
       'pitch': current_orientation.pitch,
       'yaw': current_orientation.yaw
   }
   ```

3. **Apply Offset to Future Readings:**
   ```python
   corrected_orientation = raw_orientation - zero_reference
   ```

4. **Reset Accumulated Drift:**
   ```python
   accumulated_drift = 0.0
   ```

### ROS2 Integration

For full ROS2 integration, add:

1. **Create Calibration Service:**
   ```python
   from std_srvs.srv import Trigger
   
   self.imu_calibrate_client = self.create_client(
       Trigger, 
       '/imu/calibrate'
   )
   ```

2. **Call Service in API:**
   ```python
   request = Trigger.Request()
   future = self.imu_calibrate_client.call_async(request)
   
   if future.result().success:
       return success_response
   ```

## Calibration Best Practices

### Before Calibration

1. **Place robot on level surface**
2. **Ensure robot is stationary**
3. **Allow IMU to warm up (30 seconds)**
4. **Avoid vibrations or movement**

### During Calibration

1. **Do not move the robot**
2. **Wait for calibration to complete**
3. **Avoid nearby electromagnetic interference**

### After Calibration

1. **Verify orientation readings are reasonable**
2. **Check that zero position shows (0°, 0°, 0°) or expected values**
3. **Test with small movements**
4. **Re-calibrate if readings seem incorrect**

## Integration with Other Features

### Sensor Monitoring

The IMU data displayed in the Sensors tab is updated every 500ms:
- **Orientation X, Y, Z** - Roll, Pitch, Yaw angles
- **Angular Velocity** - Rotation rate
- **Linear Acceleration X, Y** - Movement acceleration

### Path Planning

Accurate IMU calibration improves:
- Turn accuracy during waypoint navigation
- Heading maintenance on straight paths
- Return-to-home precision

### Status Monitoring

IMU health status is shown in the Status tab:
- Connected/Disconnected state
- Sensor health indicator
- Last calibration timestamp (planned feature)

## JavaScript Implementation

### Function: calibrateIMU()

```javascript
async function calibrateIMU() {
    try {
        const statusDiv = document.getElementById('imu-calibration-status');
        statusDiv.textContent = 'Calibrating IMU...';
        statusDiv.style.color = '#f59e0b';

        const response = await fetch(`${API_BASE}/api/robot/imu/calibrate`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' }
        });

        const result = await response.json();
        
        if (result.success) {
            statusDiv.textContent = '✓ IMU Calibrated - Zero reference set';
            statusDiv.style.color = '#10b981';
            addLog('IMU calibration completed successfully');
            
            setTimeout(() => {
                statusDiv.textContent = '';
            }, 3000);
        } else {
            statusDiv.textContent = '✗ Calibration failed: ' + result.error;
            statusDiv.style.color = '#ef4444';
            addLog('IMU calibration failed: ' + result.error);
        }
    } catch (error) {
        // Error handling...
    }
}
```

## Future Enhancements

### Planned Features

1. **Auto-Calibration on Startup**
   - Automatically calibrate when system starts
   - Configurable via settings

2. **Calibration History**
   - Store last calibration timestamp
   - Display in UI
   - Track calibration frequency

3. **Advanced Calibration Options**
   - Multi-point calibration
   - Magnetometer calibration (if available)
   - Temperature compensation

4. **Drift Detection**
   - Automatically detect when recalibration needed
   - Notify user via UI alert
   - Suggest recalibration intervals

5. **Calibration Profiles**
   - Save calibration for different surfaces
   - Quick load for common scenarios
   - Named profiles (e.g., "warehouse_floor", "outdoor")

6. **Visual Feedback**
   - Show IMU orientation as 3D model
   - Indicate when robot is level
   - Real-time tilt visualization

## Troubleshooting

### Calibration Fails

**Symptoms:** Red error message, API returns error

**Solutions:**
1. Check that robot is stationary
2. Verify IMU hardware connection
3. Check ROS2 service is running
4. Review system logs for details

### Orientation Still Incorrect After Calibration

**Symptoms:** Values don't match expected orientation

**Solutions:**
1. Re-calibrate on level surface
2. Check sensor mounting orientation
3. Verify coordinate system alignment
4. Consider hardware issue

### Calibration Button Not Responding

**Symptoms:** Nothing happens when clicking button

**Solutions:**
1. Check browser console for errors
2. Verify API server is running
3. Check network connectivity
4. Refresh the page

## API Code Location

**File:** `ros2_ws/src/my_robot_automation/scripts/rest_api_server.py`

**Lines:** 983-1013

```python
@self.app.route('/api/robot/imu/calibrate', methods=['POST'])
def calibrate_imu():
    """Calibrate/zero the IMU sensor"""
    # Implementation here...
```

## UI Code Location

**File:** `ros2_ws/src/my_robot_automation/scripts/web_robot_interface.py`

**Button:** Lines 1032-1037  
**JavaScript Function:** Lines 1765-1798

## Testing

### Manual Test Procedure

1. Open web interface at http://localhost:8080
2. Navigate to Sensors tab
3. Observe current IMU readings
4. Click "Calibrate IMU (Set Zero)" button
5. Wait for confirmation message
6. Verify status shows success (green checkmark)
7. Check that log panel shows calibration entry

### Expected Results

- Calibration completes in < 1 second
- Success message displayed
- Log entry created
- No errors in browser console
- No errors in ROS2 logs

## Conclusion

The IMU calibration feature provides essential functionality for maintaining accurate orientation tracking. The one-click calibration from the web UI makes it easy for operators to maintain sensor accuracy without requiring command-line access or technical knowledge.

The current implementation provides the UI and API framework, ready for integration with actual MPU6050 hardware calibration routines when deployed on the Raspberry Pi 5.

