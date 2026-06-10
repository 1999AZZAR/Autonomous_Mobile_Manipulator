# Digital Twin — TODO

## Overview
A real-time 3D virtual replica of the AMM robot. Mirrors physical state (position, orientation, sensors, arm joints) via WebSocket, visualizes the environment, and allows both monitoring and simulation without hardware.

---

## Phase 1: 3D Scene Foundation

### Scene Setup
- [ ] Initialize Three.js renderer, scene, camera, lighting
- [ ] Add orbit controls (zoom, pan, rotate around robot)
- [ ] Ground plane with grid (match 2D map scale: 1m grid)
- [ ] Ambient + directional lighting with shadows
- [ ] Responsive canvas (fills container, resizes on window change)
- [ ] Coordinate system: X-right, Y-forward, Z-up (match ROS2 convention)

### Robot Model
- [ ] Load/create robot chassis model (box primitive + wheels)
- [ ] 4 swivel wheels with rotation animation
- [ ] Robot body with heading indicator (arrow or colored front face)
- [ ] TF-Luna distance sensor bracket (front-mounted)
- [ ] Laser sensor housings (6 positions: LF, LB, RF, RB, BL, BR)
- [ ] Ultrasonic sensor pair (front-left, front-right)
- [ ] Line sensor indicators (bottom-mounted, 3 dots)
- [ ] IMU indicator (heading compass overlay or body color)

### Manipulator Arm
- [ ] 6DOF arm model (base rotation + 5 joints)
- [ ] Joint angle visualization (cylinder segments at each joint)
- [ ] Gripper model (2-finger parallel gripper)
- [ ] Gripper open/close animation
- [ ] Arm reach envelope wireframe (optional)

---

## Phase 2: Real-Time State Sync

### WebSocket Integration
- [ ] Connect to existing `ws://localhost:8001/ws/sensors` endpoint
- [ ] Parse sensor data stream (laser, ultrasonic, IMU, line, distance)
- [ ] Update robot position/orientation from IMU data
- [ ] Update joint angles from mega_interface data
- [ ] Handle connection drops (reconnect with backoff)

### State Mapping
- [ ] IMU heading → robot body rotation (Y-axis)
- [ ] IMU pitch/roll → tilt visualization (if applicable)
- [ ] Laser distances → sensor arc visualization (like 2D map)
- [ ] Ultrasonic distances → front sensor arcs
- [ ] Line sensors → bottom dot indicators (active/inactive)
- [ ] Gripper state → finger position animation
- [ ] Arm joint angles → arm model pose

### Position Tracking
- [ ] Integrate with `/api/robot/position` for x,y,z coordinates
- [ ] Dead reckoning position in simulation mode
- [ ] GPS offset display (if available)

---

## Phase 3: Environment

### Floor & Boundaries
- [ ] Textured floor plane (10x10m minimum)
- [ ] Wall markers or boundary visualization
- [ ] Scale reference objects (1m cubes, measurement markers)

### Obstacles
- [ ] Static obstacle placement (boxes, cylinders)
- [ ] Dynamic obstacle support (moveable in UI)
- [ ] Obstacle collision detection (highlight when robot approaches)
- [ ] Predefined obstacle sets (competition layout)

### Waypoint Paths
- [ ] Load saved paths from `/api/waypoints/paths/:id`
- [ ] Render path as 3D ribbon/line with direction arrows
- [ ] Current replay position highlight
- [ ] Waypoint markers with action indicators (gripper, tilt icons)
- [ ] Record new waypoints from 3D view (click to place)

### Objects of Interest
- [ ] Placeholder objects (cups, boxes) that can be picked
- [ ] Object position tracking (from AI detection results)
- [ ] Gripper-object interaction visualization

---

## Phase 4: Sensor Visualization

### Laser Sensors (6x)
- [ ] 3D ray casting from sensor positions
- [ ] Color-coded by distance (green=far, red=near)
- [ ] Hit point markers on obstacles/floor
- [ ] Update rate: match sensor polling (10Hz)

### Ultrasonic (2x)
- [ ] Cone-shaped beam visualization
- [ ] Distance label floating near sensor
- [ ] Alert color when below threshold

### Camera
- [ ] Camera frustum wireframe (FOV cone)
- [ ] Camera preview texture overlay (from `/api/ai/camera/snapshot`)
- [ ] YOLO detection bounding boxes in 3D space
- [ ] Camera on/off state indicator

### TF-Luna
- [ ] Single ray visualization
- [ ] Distance readout near sensor

---

## Phase 5: Control Interface

### 3D Controls
- [ ] Click-to-move: click on floor, robot drives to point
- [ ] Click-to-rotate: click with shift to set heading
- [ ] Joint sliders: 6 sliders for arm joints (0-180° each)
- [ ] Gripper toggle button
- [ ] Emergency stop button (red, prominent)

### Movement Modes
- [ ] Teleop mode: joystick/WASD controls robot in 3D
- [ ] Autonomous mode: watch robot follow AI decisions
- [ ] Replay mode: play back saved paths with animation
- [ ] Simulation mode: run without hardware, mock sensor data

### Arm Control
- [ ] Inverse kinematics: drag end-effector to target position
- [ ] Forward kinematics: individual joint sliders
- [ ] Pick/place workflow: click object → click target → arm moves

---

## Phase 6: UI Integration

### Tab Layout
- [ ] Add "Digital Twin" tab to sidebar navigation
- [ ] 3D canvas fills main content area
- [ ] Collapsible control panel (right side or overlay)
- [ ] Status bar at bottom (position, heading, speed, mode)

### HUD Overlay
- [ ] Position coordinates (x, y, z) in corner
- [ ] Heading compass rose
- [ ] Speed indicator
- [ ] AI mode badge (REPLAY/IFTTT/AI)
- [ ] Connection status (green/red dot)
- [ ] FPS counter (dev mode)

### Mini-Map Integration
- [ ] Sync 2D map view with 3D view
- [ ] Camera position indicator on 2D map
- [ ] Click on 2D map to set 3D camera target

---

## Phase 7: Simulation Engine

### Physics
- [ ] Basic kinematic model (position += velocity * dt)
- [ ] Wheel odometry simulation
- [ ] Simple collision detection (AABB vs obstacles)
- [ ] Gravity on arm (optional, for realistic arm simulation)

### Mock Sensors
- [ ] Generate realistic sensor data from 3D scene
- [ ] Laser: raycast against obstacles
- [ ] Ultrasonic: distance to nearest object in cone
- [ ] IMU: derive from kinematic state
- [ ] Line sensors: detect floor markings

### Scenario Runner
- [ ] Save/load complete scenarios (robot + obstacles + objects)
- [ ] Competition layout presets
- [ ] Time-based scenario playback
- [ ] Export scenario as JSON

---

## Phase 8: Recording & Playback

### Session Recording
- [ ] Record all state changes to JSON file
- [ ] Timestamp each frame
- [ ] Include sensor data, position, arm joints, AI decisions
- [ ] File size optimization (delta encoding)

### Playback
- [ ] Load recorded session
- [ ] Play/pause/scrub timeline
- [ ] Speed control (0.25x to 4x)
- [ ] Compare multiple recordings (split view)

### Analysis
- [ ] Path length calculation
- [ ] Time-to-completion metrics
- [ ] Sensor coverage heatmap
- [ ] Collision count

---

## Phase 9: Advanced Features

### Multi-Robot
- [ ] Support multiple robot instances
- [ ] Robot-to-robot distance visualization
- [ ] Collaborative task visualization

### AR Mode
- [ ] Webcam passthrough with 3D overlay
- [ ] Position tracking via AprilTags or ArUco markers
- [ ] Sensor beam visualization in real world

### Export
- [ ] Screenshot capture (PNG)
- [ ] Video recording (WebM)
- [ ] GLTF export of scene
- [ ] PDF report generation (with metrics)

---

## Tech Stack

| Component | Technology |
|-----------|-----------|
| 3D Engine | Three.js r168+ |
| Controls | OrbitControls, DragControls |
| Models | GLTF/GLB (Blender export) or primitives |
| WebSocket | Native ws:// (existing backend) |
| State | TypeScript + signals |
| Bundler | Vite (existing) |
| UI | Existing Carbon Design System |

---

## File Structure

```
frontend/src/
├── components/
│   ├── digital-twin.ts       # Main 3D scene controller
│   ├── twin-hud.ts           # HUD overlay (position, heading, status)
│   ├── twin-controls.ts      # Control panel (joint sliders, mode selector)
│   └── twin-sensors.ts       # Sensor visualization utilities
├── engine/
│   ├── scene.ts              # Three.js scene setup
│   ├── robot-model.ts        # Robot mesh builder
│   ├── arm-model.ts          # Manipulator arm mesh
│   ├── environment.ts        # Floor, walls, obstacles
│   ├── sensor-viz.ts         # Laser/ultrasonic/camera beams
│   ├── physics.ts            # Simple kinematic simulation
│   └── mock-sensors.ts       # Generate mock sensor data
├── state/
│   └── twin-state.ts         # Shared state (position, joints, sensors)
└── types/
    └── twin.ts               # Digital twin specific types
```

---

## Priority Order

1. **Phase 1** — 3D scene + robot model (visual foundation)
2. **Phase 2** — WebSocket state sync (connect to real robot)
3. **Phase 5** — Control interface (teleop in 3D)
4. **Phase 4** — Sensor visualization (see what robot sees)
5. **Phase 3** — Environment + obstacles (context)
6. **Phase 6** — UI integration (tab + HUD)
7. **Phase 7** — Simulation (run without hardware)
8. **Phase 8** — Recording/playback (analysis)
9. **Phase 9** — Advanced features (polish)

---

## Dependencies

```json
{
  "three": "^0.168.0",
  "@types/three": "^0.168.0",
  "gltf-transform": "^3.0.0"
}
```

Install: `npm install three @types/three`
