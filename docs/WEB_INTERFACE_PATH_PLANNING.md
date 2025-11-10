# Web Interface Path Planning Feature

**Last Updated:** 2025-11-10  
**Status:** Complete and Operational

## Overview

The web interface now includes a comprehensive Path Planning tab that allows direct robot control without requiring N8N workflows. This feature provides a user-friendly way to create, visualize, execute, and save navigation paths.

## Features Added

### 1. Waypoint Manager

**Add Custom Waypoints:**
- Manual input of X, Y, Z coordinates
- Precision control with 0.1m increments
- Real-time waypoint list display
- Individual waypoint removal capability

**Waypoint List Display:**
- Shows all added waypoints with coordinates
- Numbered sequentially for easy reference
- Click to remove individual waypoints
- Empty state indicator when no waypoints exist

### 2. Quick Pattern Templates

Pre-built path patterns for common tasks:

**Square Pattern (2x2m):**
- Four waypoints forming a square
- Useful for perimeter patrol
- Coordinates: (2,0), (2,2), (0,2), (0,0)

**Triangle Pattern:**
- Three waypoints forming a triangle
- Good for testing turning
- Coordinates: (2,0), (1,2), (0,0)

**Hexagon Pattern:**
- Six waypoints matching robot body shape
- 1.5m radius from origin
- Demonstrates complex path following

**Straight Line Pattern:**
- Two waypoints forming a 3m line
- Simple forward/backward motion test
- Coordinates: (0,0), (3,0)

### 3. Path Execution Settings

**Configurable Parameters:**
- Speed: 0.1 to 2.0 m/s (adjustable in 0.1 increments)
- Cycles: 1 to 10 repetitions
- Return to Start: Optional checkbox

**Default Values:**
- Speed: 0.5 m/s
- Cycles: 1
- Return to Start: Enabled

### 4. Path Visualization Canvas

**Visual Features:**
- 600x600 pixel canvas with grid overlay
- Origin marked with green crosshairs
- Scale: 50 pixels = 1 meter
- Real-time path rendering

**Path Display:**
- Blue connecting lines between waypoints
- Numbered waypoint markers
- Automatic distance calculation
- Updates dynamically as waypoints change

### 5. Execution Controls

**Start Patrol:**
- Sends waypoints to `/api/robot/patrol` endpoint
- Validates waypoint list before execution
- Displays execution status (Idle/Executing/Active/Failed)
- Shows progress percentage
- Calculates total path distance

**Clear All:**
- Removes all waypoints at once
- Resets visualization
- Confirms action with log entry

### 6. Navigation Controls

**Go to Point:**
- Navigate to single coordinates
- Uses current X, Y, Z input values
- Single-shot navigation (no cycles)
- Immediate execution

**Return Home:**
- Navigate to origin (0, 0, 0)
- Fixed speed of 0.5 m/s
- Single execution
- Quick reset functionality

**Pause:**
- Placeholder for future ROS2 action cancellation
- Logs informational message

**Stop:**
- Triggers emergency stop
- Immediately halts all robot motion
- Updates path status to "Stopped"

### 7. Path Save/Load System

**Save Path:**
- Store current waypoint configuration
- Requires unique path name
- Validates waypoint existence
- Stores in browser localStorage
- Includes creation timestamp

**Load Path:**
- Retrieve previously saved paths
- Restores all waypoints
- Updates visualization automatically
- Validates path existence

**Saved Paths List:**
- Shows all saved paths with names
- Displays waypoint count per path
- Quick load button (blue folder icon)
- Delete button (red trash icon)
- Persistent storage across sessions

### 8. Real-time Status Display

**Path Status Panel:**
- Current execution state
- Total waypoint count
- Progress percentage
- Total distance in meters

**Visual Indicators:**
- Status card color coding
- Dynamic updates during execution
- Clear error messages
- Success confirmations

## API Integration

### Primary Endpoint

**POST /api/robot/patrol**

Request format:
```json
{
  "waypoints": [
    {"position": {"x": 2.0, "y": 0.0, "z": 0.0}},
    {"position": {"x": 2.0, "y": 2.0, "z": 0.0}}
  ],
  "patrol_speed": 0.5,
  "patrol_cycles": 1,
  "return_to_start": true
}
```

Response format:
```json
{
  "success": true,
  "message": "Patrol started successfully"
}
```

### Error Handling

- Validates waypoint existence before API calls
- Catches network errors gracefully
- Displays user-friendly error messages
- Logs all errors to system log panel

## User Interface Elements

### CSS Classes Added

**Container Styles:**
- `.waypoint-container` - Scrollable waypoint list
- `.waypoint-item` - Individual waypoint card
- `.waypoint-info` - Waypoint details area
- `.waypoint-coords` - Coordinate display (monospace font)
- `.waypoint-actions` - Button container

**Input Styles:**
- `.input-grid` - Responsive input layout
- `.input-group` - Individual input field wrapper
- `.input-field` - Text/number input styling
- `.checkbox-group` - Checkbox with label

**Interactive Elements:**
- Hover effects on waypoint delete buttons
- Focus states on input fields
- Color-coded buttons (success, danger, warning)
- Grid-based responsive layout

### JavaScript Functions

**Core Functions:**
- `addWaypoint()` - Add waypoint from inputs
- `removeWaypoint(index)` - Delete specific waypoint
- `clearWaypoints()` - Remove all waypoints
- `updateWaypointList()` - Refresh display
- `updatePathVisualization()` - Redraw canvas

**Pattern Loaders:**
- `loadSquarePattern()`
- `loadTrianglePattern()`
- `loadHexagonPattern()`
- `loadLinePattern()`

**Execution:**
- `executePatrol()` - Start path following
- `goToWaypoint()` - Navigate to single point
- `returnHome()` - Return to origin
- `pausePatrol()` - Pause execution (planned)
- `stopPatrol()` - Emergency stop

**Storage:**
- `savePath()` - Save to localStorage
- `loadPath()` - Load from localStorage
- `deleteSavedPath(name)` - Remove saved path
- `updateSavedPathsList()` - Refresh saved paths display

**Canvas:**
- `initializeCanvas()` - Initialize on tab open
- `updatePathVisualization()` - Render path and waypoints

## Usage Guide

### Creating a Simple Path

1. Navigate to the **Path Planning** tab
2. Enter waypoint coordinates (X, Y, Z)
3. Click **Add Waypoint** for each point
4. Adjust speed and cycle settings if needed
5. Click **Start Patrol** to execute

### Using Quick Patterns

1. Click any pattern button (Square, Triangle, etc.)
2. Pattern loads automatically
3. View in visualization canvas
4. Adjust settings if desired
5. Click **Start Patrol**

### Saving a Path for Later

1. Create your waypoint path
2. Enter a descriptive name in "Path Name" field
3. Click **Save Path**
4. Path appears in saved paths list

### Loading a Saved Path

1. Enter path name in "Path Name" field
2. Click **Load Path**
3. Waypoints populate automatically
4. OR click the blue folder icon in saved paths list

### Quick Navigation

**To a specific point:**
1. Enter X, Y, Z coordinates
2. Click **Go to Point**

**Return to start:**
1. Click **Return Home**
2. Robot navigates to (0, 0, 0)

## Benefits Over N8N

### Direct Control
- No workflow creation needed
- Immediate execution
- Real-time feedback

### Visual Feedback
- See path before execution
- Understand spatial relationships
- Verify waypoint placement

### User Friendly
- Intuitive interface
- Quick pattern loading
- Easy waypoint management

### Persistent Storage
- Save frequently used paths
- Quick path switching
- No workflow export/import needed

### Independence
- Works without N8N running
- Direct API communication
- Simpler architecture

## Technical Details

### Canvas Rendering
- Grid: 50px spacing
- Scale: 50 pixels = 1 meter
- Origin: Canvas center
- Coordinate system: X right, Y up

### LocalStorage Usage
- Key: `robotPaths`
- Format: JSON object
- Per-path structure:
  ```json
  {
    "PathName": {
      "waypoints": [...],
      "created": "2025-11-10T..."
    }
  }
  ```

### Distance Calculation
- Euclidean distance between consecutive waypoints
- Formula: √((x₂-x₁)² + (y₂-y₁)²)
- Displayed in meters with 2 decimal places

## Future Enhancements

### Planned Features
1. Pause/Resume functionality (requires ROS2 action cancellation)
2. Real-time robot position overlay on canvas
3. Click-to-add waypoints on canvas
4. Path optimization algorithms
5. Obstacle overlay from sensor data
6. Export paths to file format
7. Import paths from file
8. Path validation against known obstacles

### Integration Opportunities
1. Combine with sensor monitoring
2. Show IMU orientation during execution
3. Display distance sensor warnings
4. Integrate with emergency stop system
5. Link with status monitoring

## Conclusion

The Path Planning feature provides comprehensive, independent robot control directly from the web interface. Users can now create, visualize, execute, and manage navigation paths without depending on N8N workflows, making the system more accessible and easier to use for direct robot control scenarios.

The interface includes all necessary features for practical path planning while maintaining a clean, intuitive design that follows the existing UI patterns.

