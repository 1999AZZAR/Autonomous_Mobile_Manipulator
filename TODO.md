# Digital Twin — TODO

## Overview
A real-time 3D virtual replica of the AMM robot. Mirrors physical state (position, orientation, sensors, arm joints) via WebSocket, visualizes the environment, and allows both monitoring and simulation without hardware.

---

## Phase 1: 3D Scene Foundation

### Scene Setup
- [x] Initialize Three.js renderer, scene, camera, lighting
- [x] Add orbit controls (zoom, pan, rotate around robot)
- [x] Ground plane with grid (match 2D map scale: 1m grid)
- [x] Ambient + directional lighting with shadows
- [x] Responsive canvas (fills container, resizes on window change)
- [x] Coordinate system: X-right, Y-forward, Z-up (match ROS2 convention)

### Robot Model
- [x] Load/create robot chassis model (box primitive + wheels)
- [x] 4 swivel wheels with rotation animation
- [x] Robot body with heading indicator (arrow or colored front face)
- [x] TF-Luna distance sensor bracket (front-mounted)
- [x] Laser sensor housings (6 positions: LF, LB, RF, RB, BL, BR)
- [x] Ultrasonic sensor pair (front-left, front-right)
- [x] Line sensor indicators (bottom-mounted, 3 dots)
- [x] IMU indicator (heading compass overlay or body color)

### Manipulator Arm
- [x] 6DOF arm model (base rotation + 5 joints)
- [x] Joint angle visualization (cylinder segments at each joint)
- [x] Gripper model (2-finger parallel gripper)
- [x] Gripper open/close animation
- [ ] Arm reach envelope wireframe (optional)

---

## Phase 2: Real-Time State Sync

### WebSocket Integration
- [x] Connect to existing `ws://localhost:8001/ws/sensors` endpoint
- [x] Parse sensor data stream (laser, ultrasonic, IMU, line, distance)
- [x] Update robot position/orientation from IMU data
- [x] Update joint angles from mega_interface data
- [x] Handle connection drops (reconnect with backoff)

### State Mapping
- [x] IMU heading → robot body rotation (Y-axis)
- [x] IMU pitch/roll → tilt visualization (if applicable)
- [x] Laser distances → sensor arc visualization (like 2D map)
- [x] Ultrasonic distances → front sensor arcs
- [x] Line sensors → bottom dot indicators (active/inactive)
- [x] Gripper state → finger position animation
- [x] Arm joint angles → arm model pose

### Position Tracking
- [x] Integrate with `/api/robot/position` for x,y,z coordinates
- [ ] Dead reckoning position in simulation mode
- [ ] GPS offset display (if available)

---

## Phase 3: Environment

### Floor & Boundaries
- [x] Textured floor plane (10x10m minimum)
- [x] Wall markers or boundary visualization
- [x] Scale reference objects (1m cubes, measurement markers)

### Obstacles
- [x] Static obstacle placement (boxes, cylinders)
- [ ] Dynamic obstacle support (moveable in UI)
- [ ] Obstacle collision detection (highlight when robot approaches)
- [ ] Predefined obstacle sets (competition layout)

### Waypoint Paths
- [x] Load saved paths from `/api/waypoints/paths/:id`
- [x] Render path as 3D ribbon/line with direction arrows
- [x] Current replay position highlight
- [x] Waypoint markers with action indicators (gripper, tilt icons)
- [ ] Record new waypoints from 3D view (click to place)

### Objects of Interest
- [ ] Placeholder objects (cups, boxes) that can be picked
- [ ] Object position tracking (from AI detection results)
- [ ] Gripper-object interaction visualization

---

## Phase 4: Sensor Visualization

### Laser Sensors (6x)
- [x] 3D ray casting from sensor positions
- [x] Color-coded by distance (green=far, red=near)
- [x] Hit point markers on obstacles/floor
- [x] Update rate: match sensor polling (10Hz)

### Ultrasonic (2x)
- [x] Cone-shaped beam visualization
- [x] Distance label floating near sensor
- [x] Alert color when below threshold

### Camera
- [x] Camera frustum wireframe (FOV cone)
- [ ] Camera preview texture overlay (from `/api/ai/camera/snapshot`)
- [ ] YOLO detection bounding boxes in 3D space
- [x] Camera on/off state indicator

### TF-Luna
- [x] Single ray visualization
- [x] Distance readout near sensor

---

## Phase 5: Control Interface

### 3D Controls
- [x] Click-to-move: click on floor, robot drives to point
- [x] Click-to-rotate: click with shift to set heading
- [x] Joint sliders: 6 sliders for arm joints (0-180° each)
- [x] Gripper toggle button
- [x] Emergency stop button (red, prominent)

### Movement Modes
- [x] Teleop mode: joystick/WASD controls robot in 3D
- [ ] Autonomous mode: watch robot follow AI decisions
- [x] Replay mode: play back saved paths with animation
- [ ] Simulation mode: run without hardware, mock sensor data

### Arm Control
- [ ] Inverse kinematics: drag end-effector to target position
- [x] Forward kinematics: individual joint sliders
- [ ] Pick/place workflow: click object → click target → arm moves

---

## Phase 6: UI Integration

### Tab Layout
- [x] Add "Digital Twin" tab to sidebar navigation
- [x] 3D canvas fills main content area
- [x] Collapsible control panel (right side or overlay)
- [x] Status bar at bottom (position, heading, speed, mode)

### HUD Overlay
- [x] Position coordinates (x, y, z) in corner
- [x] Heading compass rose
- [x] Speed indicator
- [x] AI mode badge (REPLAY/IFTTT/AI)
- [x] Connection status (green/red dot)
- [ ] FPS counter (dev mode)

### Mini-Map Integration
- [ ] Sync 2D map view with 3D view
- [ ] Camera position indicator on 2D map
- [ ] Click on 2D map to set 3D camera target

---

## Phase 7: Simulation Engine

### Physics
- [x] Basic kinematic model (position += velocity * dt)
- [x] Wheel odometry simulation
- [x] Simple collision detection (AABB vs obstacles)
- [ ] Gravity on arm (optional, for realistic arm simulation)

### Mock Sensors
- [x] Generate realistic sensor data from 3D scene
- [x] Laser: raycast against obstacles
- [x] Ultrasonic: distance to nearest object in cone
- [x] IMU: derive from kinematic state
- [ ] Line sensors: detect floor markings

### Scenario Runner
- [x] Save/load complete scenarios (robot + obstacles + objects)
- [x] Competition layout presets (Empty, Warehouse, Pick&Place, Maze)
- [ ] Time-based scenario playback
- [ ] Export scenario as JSON

---

## Phase 8: Recording & Playback

### Session Recording
- [x] Record all state changes to JSON file
- [x] Timestamp each frame
- [x] Include sensor data, position, arm joints, AI decisions
- [x] File size optimization (delta encoding)

### Playback
- [x] Load recorded session
- [x] Play/pause/scrub timeline
- [x] Speed control (0.25x to 4x)
- [ ] Compare multiple recordings (split view)

### Analysis
- [x] Path length calculation
- [x] Time-to-completion metrics
- [ ] Sensor coverage heatmap
- [ ] Collision count

---

## Phase 9: Advanced Features

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

### Implemented
- [x] Three.js r168 installed with @types/three
- [x] OrbitControls for camera manipulation
- [x] Primitive robot model (hexagonal chassis, wheels, arm, sensors)
- [x] State management via twin-state.ts
- [x] WebSocket integration via /ws/sensors endpoint
- [x] Sensor arc visualization (laser + ultrasonic)
- [x] Distance labels for all sensors
- [x] Camera frustum wireframe (AI mode only)
- [x] HUD overlay (position, heading, mode, connection status)
- [x] Control panel (joint sliders, gripper toggle, movement buttons)
- [x] Waypoint path visualization with direction arrows
- [x] Click-to-move interaction
- [x] Digital Twin tab in sidebar navigation
- [x] Simulation engine (kinematics, collision, mock sensors)
- [x] Scenario presets (Empty, Warehouse, Pick&Place, Maze)
- [x] Session recording with frame capture
- [x] Playback engine with speed control (0.25x-4x)
- [x] Analysis metrics (distance, time, speed, turns)

---

## File Structure

```
frontend/src/
├── components/
│   ├── digital-twin.ts       # Main 3D scene controller
│   ├── twin-hud.ts           # HUD overlay (position, heading, status)
│   ├── twin-controls.ts      # Control panel (joint sliders, mode selector)
├── engine/
│   ├── scene.ts              # Three.js scene setup
│   ├── robot-model.ts        # Robot mesh builder
│   ├── environment.ts        # Floor, walls, obstacles
│   ├── sensor-viz.ts         # Laser/ultrasonic/camera beams
│   ├── sensor-labels.ts      # Camera frustum + distance labels
│   ├── waypoint-viz.ts       # Waypoint path rendering
│   ├── physics.ts            # Kinematic simulation engine
│   ├── mock-sensors.ts       # Generate mock sensor data from 3D scene
│   ├── scenario.ts           # Scenario presets and save/load
│   ├── recording.ts          # Session recording to JSON
│   └── playback.ts           # Playback engine with speed control
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

### Installed
- [x] three@0.176.0 (latest stable)
- [x] @types/three@0.176.0
- [ ] gltf-transform (not yet needed — using primitives)
