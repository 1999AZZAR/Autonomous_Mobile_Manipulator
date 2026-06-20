# Autonomous Mobile Manipulator — Roadmap

## Legend

```
P0 = competition blocker — do first
P1 = core feature — needed for full autonomy
P2 = polish — makes it good
P3 = nice-to-have — makes it great
[x] = done    [ ] = todo
```

---

## 1. Mode State Machine (FSM)

```
States:
  IDLE         → waiting for command
  MANUAL       → direct teleop (dashboard/joystick)
  LINE_FOLLOW  → auto line tracking (PID)
  WAYPOINT     → dead-reckoning along saved path
  IFTTT        → rule-based automation engine
  AI_VISION    → YOLO detection + MLP decision
  TASK_SEQ     → chained multi-step mission
  CALIBRATE    → sensor calibration routine
  ESTOP        → emergency stop
```

[ ] P0 — Implement FSM in `main.py` with clean transitions
[ ] P0 — Priority hierarchy: ESTOP > LINE_FOLLOW > TASK_SEQ > IFTTT > AI > WAYPOINT > MANUAL
[ ] P0 — Auto-detect line presence → auto-switch to LINE_FOLLOW mode
[ ] P1 — Mode transition logging for student debugging
[ ] P2 — Dashboard mode indicator + manual override button

---

## 2. Obstacle Avoidance & Path Planning (RRT*)

**Current state:** `RRTStarPlanner` in `path_planning.py` has 2 critical bugs.
**Architecture: Mega (I/O) → Host (plan) → Mega (execute)**

```
                  ┌─────────────────────────┐
Mega sensors ────→│ sensor_data → GridMap    │
                  │ if path blocked: RRT*    │
                  │ convert path → commands  │
                  └─────────┬───────────────┘
                            │
Mega motors ←─────── turn/move commands
```

### Bug Fixes
[ ] P0 — **Sensor obstacle clearing bug** (`path_planning.py:63-96`): `hasattr()` on raw tuples instead of typed objects. Fix: separate obstacle type with `sensor_based` flag.
[ ] P0 — **RRT* nearest-neighbor O(n)** (`path_planning.py:153`): `min(list, key=...)` linear scan. Fix: KD-tree or grid-based spatial index.

### Sensor → Grid Pipeline
[ ] P0 — GridMap update loop: consume Mega sensor stream → project to grid → mark obstacles
[ ] P0 — Sensor aging: clear stale sensor readings after N cycles (dynamic obstacles move)
[ ] P1 — Multi-robot awareness: differentiate static vs dynamic obstacles
[ ] P2 — IR cone model: widen obstacle footprint based on sensor beam angle

### Replan Loop
[ ] P0 — Replace `_check_movement_safety` abort with RRT* replan from current pos to goal
[ ] P0 — Replan rate limiter (avoid replanning every cycle — only when path actually blocked)
[ ] P0 — Path → movement commands converter (waypoints → turn/move sequence)
[ ] P1 — Path smoothing: bezier/cubic spline between RRT* waypoints
[ ] P2 — Multi-query RRT*: reuse tree across replans instead of rebuilding

---

## 3. Line Follower Subsystem

Auto-engages when line sensors detect a track. PID to center on line.

[ ] P0 — PID controller for line tracking (3-sensor input → motor output)
[ ] P0 — Line detection algorithm: threshold calibration, noise rejection
[ ] P0 — Mode transition: auto-engage LINE_FOLLOW when line confidence > threshold
[ ] P0 — Mode transition: fall back to previous mode when line lost
[ ] P1 — Line junction detection (T-junction, cross, dead-end)
[ ] P1 — Line color/direction detection (red vs white tape)
[ ] P2 — Frontend: line sensor visualization + PID tuning sliders
[ ] P3 — Adaptive PID gain scheduling (speed-based)

---

## 4. Task Sequencer

Chains multi-step missions: `pick red cube from A → go to B → spin 2x → go to C → place cube`

[ ] P0 — Task sequence data model (steps, conditions, branching)
[ ] P0 — Executor: run steps sequentially, handle failures
[ ] P0 — Condition system: wait-for-sensor, wait-for-time, wait-for-position
[ ] P0 — Manipulation actions: pick, place, spin, tilt, lift
[ ] P0 — Navigation actions: goto waypoint (+obstacle avoidance), follow line, rotate
[ ] P1 — Recursive sub-tasks (call another sequence)
[ ] P1 — Retry logic with configurable attempts
[ ] P2 — Frontend: visual task builder (drag-drop blocks) — STEM education
[ ] P2 — Live task progress display on dashboard
[ ] P3 — Task import/export (share competition routines)

---

## 5. IFTTT Automation Engine (STEM Education)

Rule-based behavior for students to learn automation logic without coding.

Current: basic engine exists in `automation_engine.py` / `automation_api.py`

[ ] P1 — Add triggers: line_detected, object_spotted, battery_low, button_press, path_blocked
[ ] P1 — Add actions: follow_line, record_path, run_task, play_sound, send_log, emergency_stop
[ ] P1 — Condition chaining: AND/OR/NOT logic for rules
[ ] P1 — Rule priority / conflict resolution when multiple rules trigger
[ ] P2 — Frontend: visual rule builder (trigger → condition → action cards)
[ ] P2 — Live automation log: show which rules fired and why
[ ] P3 — Rule templates for common competition patterns
[ ] P3 — Export/import rule sets for classroom sharing

---

## 6. AI Vision + Offline ML Pipeline

Object detection via YOLO + decision via local MLP. Fully offline.

[ ] P1 — YOLOv8 ONNX export + optimized inference
[ ] P1 — Feature encoder: detection → 125-dim feature vector (improve if needed)
[ ] P1 — MLP inference hardening: confidence thresholds, fallback strategy
[ ] P1 — Recursive/local model: update behavior based on repeated patterns
[ ] P1 — Camera: power management (on/off by mode), auto-exposure
[ ] P2 — Training pipeline: collect data → train → export → deploy
[ ] P2 — Simulation training: use digital twin for synthetic data generation
[ ] P2 — Frontend: AI vision tab with detection overlay + confidence display
[ ] P3 — Multi-object tracking (keep object identity across frames)

---

## 7. Manipulation System

Gripper / tilt / lifter control for pick-and-place.

[ ] P1 — Gripper: force feedback (stall detection via current sense)
[ ] P1 — Lifter: position control with encoder feedback
[ ] P1 — Tilt: servo sync with camera view
[ ] P2 — Pick-place sequence: approach → grip → lift → nav → lower → release
[ ] P2 — Object size estimation from camera bounding box
[ ] P3 — Multi-object sorting (by color, shape, size)

---

## 8. Waypoint Navigation (IMU Dead Reckoning)

Dead-reckoning along saved paths. Used when no obstacles present.

[ ] P1 — Path smoothing: bezier/linear interpolation between waypoints
[ ] P1 — Obstacle override: if sensor sees blocked path, hand off to RRT* replan
[ ] P1 — IMU drift compensation: periodic zero-velocity updates
[ ] P2 — Frontend: draw path on map, edit waypoints
[ ] P2 — Path recording: drive once, save, replay
[ ] P3 — SLAM integration for drift correction

---

## 9. Arduino Mega Firmware

Mega = I/O device: reads sensors, executes motor commands. All planning on host.

[x] P2 — Mega firmware refinement: loop() speed (VFF throttled 20ms), -581 lines dead code, APF direction fix
[ ] P2 — Encoder velocity filter (low-pass for smoother readings)
[ ] P3 — Watchdog timer (auto-stop on serial timeout)
[ ] P3 — Current sensing for gripper stall detection

---

## 10. Competition Features

[ ] P2 — Clean restart: one-button start, auto-calibrate
[ ] P2 — Emergency stop: hardware button + dashboard + remote
[ ] P2 — Lap timing / mission timer
[ ] P2 — Battery monitoring: voltage check, low-battery behavior
[ ] P2 — Autonomous restart after failure (retry with fallback)
[ ] P3 — Competition logging: full sensor/motor log for post-mortem
[ ] P3 — Remote scoring integration

---

## 11. Frontend Dashboard Polish

[ ] P2 — Competition HUD: mode, battery, timer, task progress
[ ] P2 — Live obstacle map: occupancy grid overlay on map view
[ ] P2 — Line follower tuning panel (PID gains, threshold sliders)
[ ] P2 — Task builder: drag-drop sequence blocks
[ ] P2 — IFTTT rule builder: trigger → condition → action cards
[ ] P2 — AI vision overlay: bounding boxes + labels on live feed
[ ] P3 — Robot telemetry charts (real-time sensor graphs)
[ ] P3 — Path editor: click-to-add waypoints on map
[ ] P3 — Responsive layout for tablet control

---

## 12. Infrastructure & Calibration

[ ] P2 — Line sensor calibration: auto-detect black/white thresholds
[ ] P2 — IMU calibration: gyro bias, accelerometer alignment
[ ] P2 — Motor calibration: encoder ticks per mm, wheel alignment
[ ] P2 — Startup script: auto-detect HW, fall back to sim mode
[ ] P3 — Docker compose: remove unused services (post-gazebo cleanup)
[ ] P3 — OTA firmware update for Arduino Mega
[ ] P3 — System health dashboard (CPU, RAM, temp, disk)

---

## Priority Playbook

```
SPRINT 1 (Obstacle Avoidance MVP)
├── Fix RRT* sensor clearing bug (@path_planning.py:63)
├── Fix RRT* O(n) nearest-neighbor (@path_planning.py:153)
├── Implement sensor→GridMap update loop
├── Replace abort-on-obstacle with RRT* replan (@app.py:1124)
└── Wire into main.py FSM

SPRINT 2 (Navigation)
├── FSM state machine
├── Line follower PID
├── Task sequencer basic
├── Path smoothing
└── Competition start/stop/estop

SPRINT 3 (Full Autonomy)
├── IFTTT rule engine upgrades
├── ML pipeline ONNX hardening
├── Manipulation pick/place
└── Waypoint replay with obstacle handoff

SPRINT 4 (STEM Education)
├── Visual task builder (frontend)
├── IFTTT visual rule builder
├── Competition HUD + obstacle map
└── Calibration tools + documentation
```
