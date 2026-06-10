# Web Interface

TypeScript frontend served by nginx on port 3000. Proxies `/api/` and `/ws/` to the Flask backend on port 8001.

## Access

- Local: `http://localhost:3000`
- Remote: `http://<robot-ip>:3000`

## Tabs

| Tab | Component | Purpose |
|-----|-----------|---------|
| Dashboard | `dashboard.ts` | Overview: mode, sensors, recent activity |
| Movement | `movement.ts` | 8-direction pad, speed slider, turbo toggle |
| Manipulator | `manipulator.ts` | Gripper open/close, tilt angle, lifter up/down |
| Map | `map-view.ts` | 2D canvas map: robot position, waypoint paths, sensor arcs, drawing mode, training mode |
| Digital Twin | `digital-twin.ts` | 3D Three.js scene: robot model, simulation, recording, playback |
| Automations | `automations.ts` | IFTTT rule builder: trigger/action/test, enable/disable |
| AI Control | `ai-control.ts` | Camera preview, task goal, start/stop AI, decision history, waypoint manager |
| System | `system.ts` | System status, serial monitor, logs |

## Emergency Stop

Fixed button (bottom-right corner), always visible. Calls `POST /api/robot/emergency-stop`.

## 2D Map Features

- **Navigate mode**: Click to set waypoint, robot navigates via IMU dead reckoning
- **Drawing mode**: Draw walls (orange lines) and obstacles (brown boxes) on the canvas
- **Training mode**: Records laser/ultrasonic sensor data to build an occupancy grid (red heatmap)
- **Legend**: Robot (green), Waypoints (yellow), Lasers (green arcs), Ultrasonic (blue arcs), Walls (orange), Obstacles (brown), Trained (red)

## Digital Twin

Three.js 3D visualization with:
- Hexagonal robot chassis, 3 omni wheels, heading arrow
- Gripper assembly (tilt servo, fingers, camera, TF-Luna)
- Lifter with vertical movement and limit indicators
- Sensor arc visualization (laser green, ultrasonic blue, TF-Luna magenta)
- Simulation mode with physics, mock sensors, and scenario presets
- Recording and playback with speed control
- Click-to-move via raycasting on ground plane

## Technology

- Vite build system
- TypeScript (strict mode)
- Carbon Design System dark theme
- Phosphor Icons
- IBM Plex Sans/Mono fonts
- Three.js for 3D rendering

## File Location

```
frontend/
├── src/
│   ├── main.ts
│   ├── api.ts
│   ├── types.ts
│   ├── components/     # UI panels
│   ├── engine/         # Three.js digital twin
│   └── state/          # Shared state
└── index.html
```
