# Offline AI Decision System — YOLO + MLP

## Overview

Replace the OpenAI-dependent AI flow with a fully offline pipeline: YOLOv8 (object detection) feeds into an MLP (Multi-Layer Perceptron) that outputs movement commands. The MLP is trained in simulation mode using the existing sensor + YOLO data pipeline. Trained rules are auto-exported as IFTTT automations for the rule-based engine. Map and digital twin are interconnected for shared world state.

**Zero external API dependency. Everything runs on the Raspberry Pi.**

---

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    SENSOR INPUT LAYER                        │
│                                                              │
│  Real Mode:  Mega serial → SensorManager → flat keys        │
│  Sim Mode:   MockSensorGenerator → same flat keys           │
│              (physics.step → raycast → MockSensorData)       │
└──────────────────────┬──────────────────────────────────────┘
                       │
┌──────────────────────▼──────────────────────────────────────┐
│                    FEATURE VECTOR                            │
│                                                              │
│  [laser×6, ultra×2, line×3, imu×3, yolo×N, heading, vel]   │
│  Normalized to [0,1] range per sensor type                  │
│  Window of last W frames (default W=5) for temporal context │
└──────────────────────┬──────────────────────────────────────┘
                       │
          ┌────────────┴────────────┐
          │                         │
┌─────────▼──────────┐  ┌──────────▼─────────────┐
│   YOLOv8nano       │  │   MLP Decision Engine   │
│   (object detect)  │  │   (movement command)     │
│                    │  │                          │
│  Input: camera     │  │  Input: feature_vector   │
│  Output: classes,  │  │  Output: command probs   │
│  confidence, bbox  │  │  (f/b/q/e/z/x/t/y/s)    │
└─────────┬──────────┘  └──────────┬─────────────┘
          │                         │
          └────────────┬────────────┘
                       │
┌──────────────────────▼──────────────────────────────────────┐
│                 DECISION FUSION                              │
│                                                              │
│  1. YOLO detects obstacle/person → override to STOP         │
│  2. Sensor critical zone → override to STOP                 │
│  3. MLP command → execute if safe                           │
│  4. Confidence threshold → fallback to rule-based           │
└──────────────────────┬──────────────────────────────────────┘
                       │
┌──────────────────────▼──────────────────────────────────────┐
│                 OUTPUT LAYER                                 │
│                                                              │
│  1. Execute command via mega_interface.send_command()        │
│  2. Log decision to AiDecision table (sensor + yolo + cmd)  │
│  3. Auto-generate IFTTT rule if pattern repeats ≥ K times   │
│  4. Update twin/map state                                   │
└─────────────────────────────────────────────────────────────┘
```

---

## Phase 1: Feature Engineering & Data Pipeline

**Goal:** Build the normalized feature vector that feeds both training and inference.

### 1.1 — Feature Vector Definition

- [ ] Create `ros2_ws/src/my_robot_automation/scripts/ml/feature_encoder.py`
  - Class `FeatureEncoder`:
    - `encode(sensor_data, yolo_detections, robot_state) → np.ndarray`
    - Sensor features (14 dims): 6 laser + 2 ultra + 3 line + imu_heading + imu_pitch + imu_roll
    - YOLO features (8 dims): max 4 objects × 2 (class_onehot[4] + confidence + distance_estimate)
    - Robot state (3 dims): current_vx, current_vy, current_omega (from last command)
    - Temporal context: stack last W=5 frames → total = 14+8+3 = 25 × 5 = **125 dims**
  - Normalization:
    - Laser: `/ 1500.0` (max range mm)
    - Ultra: `/ 4000.0` (max range mm)
    - Line: already 0/1
    - IMU: `/ 180.0` (degrees to [-1,1])
    - YOLO confidence: already [0,1]
    - Velocity: `/ 500.0` (max speed mm/s)
  - Save/load scaler params to JSON for consistent inference

### 1.2 — Training Data Collection

- [ ] Create `ros2_ws/src/my_robot_automation/scripts/ml/training_collector.py`
  - Class `TrainingCollector`:
    - `record_frame(features: np.ndarray, command: str, outcome: dict)`
    - Stores to SQLite DB: `ml_training_data` table
      - columns: `id, timestamp, features(BLOB), command(str), reward(float), session_id(str), mode(str)`
    - `mode`: `"real"` or `"simulation"`
    - `reward`: computed from outcome (see 1.3)
    - Sessions: each training run gets a UUID session_id for filtering
  - Auto-prune: keep last 100k frames, delete oldest sessions first

### 1.3 — Reward Function

- [ ] Create `ros2_ws/src/my_robot_automation/scripts/ml/reward.py`
  - Class `RewardCalculator`:
    - `compute(sensor_before, sensor_after, command, yolo_detections) → float`
    - Reward components:
      - **Progress reward**: +1.0 if robot moved toward goal (waypoint or open space)
      - **Collision penalty**: -2.0 if any sensor < CRITICAL threshold
      - **Stagnation penalty**: -0.1 per frame if position unchanged for >30 frames
      - **Obstacle avoidance bonus**: +0.5 if robot turned away from detected obstacle
      - **Object interaction bonus**: +1.0 if robot approached detected object of interest
      - **Line following bonus**: +0.3 if line sensors detected while following
    - Reward range: roughly [-2.0, +2.0] per frame

### 1.4 — Data Format Validation

- [ ] Add `/api/ml/data` endpoints to backend
  - `GET /api/ml/data/stats` — total frames, sessions, command distribution
  - `GET /api/ml/data/export?format=csv` — export training data
  - `POST /api/ml/data/import` — import training data from file
  - `DELETE /api/ml/data/session/<id>` — delete a training session

---

## Phase 2: MLP Model

**Goal:** Train and serve an MLP that maps feature vectors to movement commands.

### 2.1 — Model Architecture

- [ ] Create `ros2_ws/src/my_robot_automation/scripts/ml/mlp_model.py`
  - Class `MLPDecisionModel`:
    - Architecture (PyTorch):
      ```
      Input(125) → Linear(128) → ReLU → Dropout(0.2)
                → Linear(64) → ReLU → Dropout(0.2)
                → Linear(32) → ReLU
                → Linear(9) → Softmax
      ```
    - 9 output classes: `['f', 'b', 'q', 'e', 'z', 'x', 't', 'y', 's']` (the 8 moves + stop)
    - `predict(feature_vector) → (command: str, confidence: float, probs: dict)`
    - `predict_with_override(feature_vector, yolo_detections, sensor_data) → (command, confidence, overrides)`
      - YOLO override: if person/cat detected (conf≥0.4) → force 's' (stop)
      - Sensor override: if any sensor < CRITICAL → force 's' (stop)
      - Sensor warning: if sensor < WARNING → boost stop probability by 0.3
    - Model file: `models/mlp_decision.pt`
    - Config file: `models/mlp_config.json` (architecture params, scaler params, class labels)

### 2.2 — Training Pipeline

- [ ] Create `ros2_ws/src/my_robot_automation/scripts/ml/trainer.py`
  - Class `MLPTrainer`:
    - `train(data_loader, epochs=50, lr=0.001, batch_size=64) → TrainingResult`
      - Train/val split: 80/20 stratified by command class
      - Loss: CrossEntropyLoss (command classification)
      - Optimizer: Adam with weight decay 1e-4
      - LR scheduler: ReduceLROnPlateau (patience=5)
      - Early stopping: val_loss not improving for 10 epochs
    - `evaluate(test_data) → EvalMetrics`
      - Accuracy, per-class precision/recall, confusion matrix
    - `save_model(path)` / `load_model(path)`
    - `export_onnx(path)` — export to ONNX for faster inference on Pi
    - Training history logged to `models/training_history.json`

### 2.3 — Simulation Training Mode

- [ ] Create `ros2_ws/src/my_robot_automation/scripts/ml/sim_trainer.py`
  - Class `SimulationTrainer`:
    - Connects to frontend simulation loop via shared state
    - `start_training(scenario, episodes=100, strategy='epsilon-greedy')`
      - Strategy: epsilon-greedy (epsilon decays from 1.0 to 0.05 over training)
      - Each episode:
        1. Reset robot to scenario start position
        2. MLP predicts command → physics.step() → mockSensors.generate()
        3. Reward calculated from sensor changes + position progress
        4. Frame recorded to training DB
        5. Episode ends: max_steps reached OR collision OR goal reached
      - After each episode: batch update MLP weights from collected frames
    - `get_training_stats() → dict`
      - episodes_completed, avg_reward, avg_steps, epsilon, best_episode_reward
    - Frontend UI: training progress bar, reward chart, epsilon display

### 2.4 — Backend API for Model Serving

- [ ] Add endpoints to `automation_api.py`
  - `POST /api/ml/model/train` — start training (body: `{episodes, scenario, strategy}`)
  - `GET /api/ml/model/status` — training status + stats
  - `POST /api/ml/model/stop` — stop training
  - `GET /api/ml/model/metrics` — accuracy, confusion matrix, training history
  - `POST /api/ml/model/predict` — single prediction (body: `{features}`)
  - `GET /api/ml/model/export` — download model weights (.pt or .onnx)
  - `POST /api/ml/model/load` — upload and load model weights

---

## Phase 3: IFTTT Auto-Generation

**Goal:** When the MLP discovers a repeated sensor→action pattern, auto-generate an IFTTT automation rule.

### 3.1 — Pattern Extraction

- [ ] Create `ros2_ws/src/my_robot_automation/scripts/ml/pattern_extractor.py`
  - Class `PatternExtractor`:
    - `analyze_session(session_id) → List[Pattern]`
      - Groups consecutive frames with same command
      - Extracts sensor conditions: `if laser_left_front < X AND ultra_front_left < Y → command Z`
      - Conditions: take min/max of sensor values during the pattern window
      - Pattern: `{conditions: [{feed, op, threshold}], action: command, confidence, count}`
    - `find_recurring_patterns(min_count=5) → List[RecurringPattern]`
      - Searches across all sessions for patterns that appear ≥ min_count times
      - Computes confidence = occurrence_count / total_applicable_frames
    - `suggest_automation(pattern) → AutomationDraft`
      - Converts pattern to IFTTT format:
        - `triggerType: 'sensor'`
        - `conditionMatch: 'ALL'`
        - `conditions: pattern.conditions`
        - `actions: [{type: 'move', value: pattern.action}]`

### 3.2 — Auto-Rule Creation

- [ ] Create `ros2_ws/src/my_robot_automation/scripts/ml/rule_generator.py`
  - Class `RuleGenerator`:
    - `generate_from_patterns(patterns, min_confidence=0.7) → List[AutomationDraft]`
    - `apply_draft(draft) → Automation` — creates automation via Prisma
    - `dry_run(draft) → DryRunResult` — simulates rule against historical data
    - Rate limiting: max 1 new rule per 10 minutes
    - Deduplication: skip if rule already exists with same conditions + action
  - Frontend notification: "New rule suggested: If laser_left_front < 200mm then Turn Right"

### 3.3 — Rule Management UI

- [ ] Extend `frontend/src/components/automations.ts`
  - Add "AI-Suggested Rules" section
  - Show suggested rules with confidence score
  - One-click "Apply" or "Dismiss" buttons
  - Show which training session generated the rule
  - History of auto-generated rules

---

## Phase 4: Simulation Sensor Fidelity

**Goal:** Ensure mock sensors produce realistic, training-quality data.

### 4.1 — Enhanced Mock Sensors

- [ ] Update `frontend/src/engine/mock-sensors.ts`
  - Add sensor noise model: `actual = reading + gaussian_noise(0, sigma)`
    - Laser sigma: 15mm (1% of max range)
    - Ultra sigma: 30mm (0.75% of max range)
    - Line sensors: flip probability 0.02 at threshold boundary
    - IMU: heading drift 0.5°/minute random walk
  - Add sensor dropout simulation: 2% chance of sensor returning max_range
  - Add occlusion: if obstacle blocks sensor, return distance to obstacle (already works via raycast)
  - Add crosstalk: ultrasonic sensors influence each other when < 200mm apart

### 4.2 — Simulation ↔ Real Data Parity

- [ ] Create `ros2_ws/src/my_robot_automation/scripts/ml/data_validator.py`
  - Class `DataValidator`:
    - `compare_distributions(real_data, sim_data) → DistributionReport`
      - Compare histogram of each sensor type
      - KS test for distribution similarity
      - Report which sensors diverge and by how much
    - `calibrate_sim_to_real(real_session_ids) → CalibrationParams`
      - Fit scaling factors per sensor: `sim_adjusted = sim * factor + offset`
      - Save calibration to `models/sim_calibration.json`

### 4.3 — Simulation Training Integration

- [ ] Update `frontend/src/components/digital-twin.ts`
  - When `simMode === 'simulation'` and AI training active:
    - Each sim tick: encode features → MLP predict → execute → record reward
    - Expose training state via `getTrainingState() → {episode, step, reward, epsilon}`
    - Training progress rendered in twin HUD overlay
  - When training in sim, also feed data to backend training DB via `POST /api/ml/data/record`

---

## Phase 5: Inference Pipeline

**Goal:** Replace `AIDecisionEngine._call_ai_api()` with local MLP inference.

### 5.1 — Offline Decision Engine

- [ ] Create `ros2_ws/src/my_robot_automation/scripts/ml/inference_engine.py`
  - Class `OfflineDecisionEngine`:
    - `__init__(model_path, feature_encoder, yolo_model)`
    - `analyze(sensor_data, camera_frame, task_goal) → DecisionResult`
      1. Run YOLO on camera frame → detections
      2. Encode features from sensor_data + detections
      3. MLP predict → command + confidence
      4. Apply safety overrides (YOLO person → stop, critical sensor → stop)
      5. If confidence < 0.4 → fallback to rule-based (`_rule_based_from_yolo`)
      6. Log to AiDecision table
      7. Return `DecisionResult(reasoning, actions, confidence, mode='offline_ai')`
    - `get_model_info() → dict` — model version, accuracy, last training date

### 5.2 — Integration with Existing AI Loop

- [ ] Update `ros2_ws/src/my_robot_automation/scripts/ai_decision.py`
  - Add `mode = 'offline_ai'` to mode selection
  - `_select_mode()`: check if `offline_ai` model is loaded → use it
  - Keep existing modes: `replay`, `ifttt`, `ai` (API), `offline_ai` (MLP)
  - `_run_analysis()`: if mode is `offline_ai`:
    - Call `OfflineDecisionEngine.analyze()` instead of `_call_ai_api()`
    - No HTTP request, no API key, fully offline
  - Add config: `AI_BACKEND=offline_ai` env var

### 5.3 — Frontend AI Tab Update

- [ ] Update `frontend/src/components/ai-control.ts`
  - Add "Offline AI" backend option in selector
  - Show model info: accuracy, training date, total training frames
  - Show MLP prediction confidence as gauge
  - Show YOLO detections + MLP decision side by side
  - "Train Model" button → opens training panel

---

## Phase 6: Map ↔ Twin Interconnection

**Goal:** 2D map and 3D twin share world state and stay synchronized.

### 6.1 — Shared World State

- [ ] Create `frontend/src/state/world-state.ts`
  - Singleton `WorldState`:
    - `robotPosition: {x, y, heading}` — single source of truth
    - `occupancyGrid: Float32Array(60×50)` — from map training
    - `obstacles: Obstacle[]` — shared between map drawing and twin obstacles
    - `waypoints: Waypoint[]` — shared paths
    - `sensorRays: SensorRay[]` — laser/ultra readings with origin + endpoint
    - `listeners: Map<string, callback>` — components subscribe to changes
    - `update(source, data)` — any component can update, all others notified
  - Sources: `'sim'`, `'real'`, `'map-draw'`, `'twin-interact'`

### 6.2 — Map ↔ Twin Sync

- [ ] Update `frontend/src/components/map-view.ts`
  - Import `WorldState`, subscribe to position changes
  - Robot icon position comes from `WorldState.robotPosition`
  - Occupancy grid updates propagate to twin (add obstacle meshes)
  - Drawing walls/obstacles in map → adds Three.js meshes to twin scene
  - Twin click-to-move → updates map robot position

- [ ] Update `frontend/src/components/digital-twin.ts`
  - Import `WorldState`, subscribe to position changes
  - Sim loop writes to `WorldState` each tick
  - Real mode polling writes to `WorldState` each fetch
  - Map-drawn obstacles appear in 3D scene
  - Camera follows robot option (sync camera to WorldState.position)

### 6.3 — Occupancy Grid in 3D

- [ ] Update `frontend/src/engine/environment.ts`
  - Add `updateOccupancyGrid(grid: Float32Array)` method
  - Render occupied cells as semi-transparent red cubes at z=0.01
  - Render free cells as semi-transparent green (optional, toggle)
  - Updates when WorldState.occupancyGrid changes

---

## Phase 7: Full Training Loop

**Goal:** End-to-end training workflow from UI to deployed model.

### 7.1 — Training Scenarios

- [ ] Create `ros2_ws/src/my_robot_automation/scripts/ml/scenarios.py`
  - `SCENARIOS`:
    - `empty_room`: no obstacles, learn basic movement
    - `obstacle_course`: static obstacles, learn avoidance
    - `object_pickup`: objects placed, learn approach + gripper
    - `line_follow`: floor markings, learn line tracking
    - `free_explore`: random open space, learn diverse behaviors
  - Each scenario defines: robot start pose, obstacle layout, object placements, goal conditions, max steps

### 7.2 — Training UI

- [ ] Add "ML Training" panel to AI tab or new "Training" tab
  - Scenario selector dropdown
  - Training parameters: episodes, learning rate, epsilon
  - Start/Stop/Pause buttons
  - Live metrics: episode count, avg reward, epsilon, current step
  - Reward chart (Chart.js or canvas)
  - Confusion matrix heatmap
  - Download model button
  - Training history list (sessions with metrics)

### 7.3 — Continuous Learning

- [ ] Add `online_learning` mode
  - During normal real-mode operation, record every decision + outcome
  - Periodically (every N frames or on explicit trigger) retrain MLP with new data
  - A/B test: run old model alongside new, compare rewards
  - Auto-promote new model if it outperforms old by >5%

---

## File Structure (New Files)

```
ros2_ws/src/my_robot_automation/scripts/ml/
├── __init__.py
├── feature_encoder.py      # Feature vector construction + normalization
├── training_collector.py   # SQLite training data storage
├── reward.py               # Reward function computation
├── mlp_model.py            # PyTorch MLP model definition
├── trainer.py              # Training loop + evaluation
├── sim_trainer.py          # Simulation training orchestrator
├── inference_engine.py     # Offline decision engine (replaces API)
├── pattern_extractor.py    # Extract repeated sensor→action patterns
├── rule_generator.py       # Auto-generate IFTTT rules from patterns
├── data_validator.py       # Compare real vs sim sensor distributions
└── scenarios.py            # Training scenario definitions

frontend/src/
├── engine/
│   └── occupancy-grid.ts   # 3D occupancy grid renderer (for twin)
├── state/
│   └── world-state.ts      # Shared world state singleton
├── components/
│   └── ml-training.ts      # ML training UI panel

models/
├── mlp_decision.pt         # PyTorch model weights
├── mlp_config.json         # Model architecture + scaler params
├── sim_calibration.json    # Sim-to-real calibration factors
└── training_history.json   # Training run metrics
```

---

## Dependencies (Python — all offline)

```txt
# Already in ROS2 base
numpy
scikit-learn          # data splitting, metrics

# Need to install
torch>=2.0            # MLP training + inference (CPU-only on Pi)
onnxruntime           # optional: faster inference via ONNX
ultralytics           # YOLOv8 (already used)
```

Frontend: no new dependencies (reuse Three.js, existing UI).

---

## Implementation Order

| # | Phase | Effort | Prerequisite |
|---|-------|--------|-------------|
| 1 | Feature Engineering & Data Pipeline | 2-3 days | — |
| 2 | MLP Model (architecture + training) | 3-4 days | Phase 1 |
| 3 | Simulation Sensor Fidelity | 1-2 days | — |
| 4 | Simulation Training Mode | 2-3 days | Phase 2, 3 |
| 5 | IFTTT Auto-Generation | 2-3 days | Phase 2 |
| 6 | Inference Pipeline | 1-2 days | Phase 2 |
| 7 | Map ↔ Twin Interconnection | 2-3 days | — |
| 8 | Full Training Loop + UI | 3-4 days | Phase 4, 5, 6 |

**Total estimated: 16-22 days**

Parallel tracks:
- Phase 1 + 3 + 7 can run in parallel (no dependencies)
- Phase 2 depends on Phase 1
- Phase 4 depends on Phase 2 + 3
- Phase 5 depends on Phase 2
- Phase 6 depends on Phase 2
- Phase 8 depends on Phase 4 + 5 + 6

---

## Key Design Decisions

1. **PyTorch over TensorFlow**: lighter for Pi deployment, better ONNX export, simpler API for small models
2. **MLP over RNN/LSTM**: 125-dim feature vector with temporal window gives sufficient context without sequential complexity; MLP trains faster on limited data
3. **9-class classification** (not regression): discrete commands match existing serial protocol, simpler reward shaping
4. **Epsilon-greedy over pure policy gradient**: simpler to implement, works well for discrete action spaces, easy to tune exploration
5. **SQLite for training data**: already available on Pi, no extra services, sufficient for 100k frames
6. **ONNX export option**: PyTorch model exported to ONNX can use onnxruntime for 2-3x faster inference on ARM CPU
7. **Reward shaping over pure RL**: explicit reward components guide learning faster than undirected exploration
8. **Auto-IFTTT bridge**: MLP patterns become deterministic rules → faster execution for known situations, MLP handles novel ones
