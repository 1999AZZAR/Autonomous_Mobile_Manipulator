// Digital Twin — main 3D scene controller

import { createScene, type TwinScene } from '../engine/scene';
import { createRobotModel, updateGripper, updateTiltServo, updateLifter, updateLineSensors, updateWheelRotation } from '../engine/robot-model';
import { createEnvironment } from '../engine/environment';
import { createLaserArcs, createUltrasonicCones, createTfLunaRay, updateSensorArcs, addLaserArcsToScene, addUltrasonicConesToScene } from '../engine/sensor-viz';
import { createPathLine, createReplayMarker } from '../engine/waypoint-viz';
import { createCameraFrustum, createSensorLabel, updateSensorLabel } from '../engine/sensor-labels';
import { scenarioToObstacles, PRESETS, type Scenario } from '../engine/scenario';
import { SessionRecorder, saveRecording, type Recording } from '../engine/recording';
import { PlaybackEngine, type PlaybackProgress } from '../engine/playback';
import { onTwinStateChange, getTwinState, updateTwinState } from '../state/twin-state';
import { fetchRobotPosition, fetchSensors, fetchPath, sendContinuousCommand, syncSimObstacles, fetchSimObstacles, sendNavigationGoal } from '../api';
import { processArduinoMega, type VffResult } from '../engine/arduino-mega';
import type { RobotModelParts, Obstacle } from '../types/twin';
import type { SensorArcMesh } from '../engine/sensor-viz';
import { PhysicsEngine } from '../engine/physics';
import { planRRT, type RRTResult, type Point } from '../engine/rrt';
import { WorldState } from '../state/world-state';
import { createOccupancyGridRenderer, type OccupancyGridRenderer } from '../engine/occupancy-grid';
import * as THREE from 'three';

let scene: TwinScene | null = null;
let robotGroup: any = null;
let robotParts: RobotModelParts | null = null;
let laserArcs: Map<string, SensorArcMesh> | null = null;
let ultraCones: Map<string, SensorArcMesh> | null = null;
let hitPoints: Map<string, any> | null = null;
let tfLuna: { line: any; hitPoint: any } | null = null;
let unsubscribe: (() => void) | null = null;
let containerEl: HTMLElement | null = null;
let cameraFrustum: any = null;
let sensorLabels: Map<string, any> | null = null;
let currentPathGroup: any = null;
let replayMarker: any = null;
let simMode: 'real' | 'simulation' = 'real';
let simInterval: ReturnType<typeof setInterval> | null = null;
let realPollInterval: ReturnType<typeof setInterval> | null = null;
let recorder: SessionRecorder | null = null;
let playback: PlaybackEngine | null = null;
let occGrid: OccupancyGridRenderer | null = null;

let targetMarker: THREE.Mesh | null = null;

// ── RRT* state ──
let rrtObstacles: Obstacle[] = [];
let rrtResult: RRTResult | null = null;
let rrtPathMesh: THREE.Line | null = null;
let rrtTreeGroup: THREE.Group | null = null;

function initHitPoints() {
  hitPoints = new Map<string, THREE.Mesh>();
  laserArcs?.forEach((arc, key) => {
    hitPoints!.set(key, arc.hitPoint);
  });
  ultraCones?.forEach((cone, key) => {
    hitPoints!.set(key, cone.hitPoint);
  });
}

export function initDigitalTwin(container: HTMLElement) {
  containerEl = container;

  scene = createScene(container);

  const model = createRobotModel();
  robotGroup = model.group;
  robotParts = model.parts;
  scene.scene.add(robotGroup);

  createEnvironment(scene.scene);

  occGrid = createOccupancyGridRenderer(scene.scene);
  occGrid.setVisible(false);
  WorldState.subscribe('digital-twin', (state) => {
    if (occGrid) {
      occGrid.updateGrid(state.occupancyGrid, state.gridWidth, state.gridHeight, state.gridOriginX, state.gridOriginY);
    }
  });

  laserArcs = createLaserArcs();
  ultraCones = createUltrasonicCones();
  tfLuna = createTfLunaRay();

  addLaserArcsToScene(laserArcs, scene.scene);
  addUltrasonicConesToScene(ultraCones, scene.scene);
  scene.scene.add(tfLuna.line);
  scene.scene.add(tfLuna.hitPoint);

  cameraFrustum = createCameraFrustum();
  scene.scene.add(cameraFrustum);

  sensorLabels = new Map();
  const labelPositions: Record<string, THREE.Vector3> = {
    laser_left_front:  new THREE.Vector3(-0.19,  0.045, 0.22), // IR Left 1  (A0) — left face
    laser_left_back:   new THREE.Vector3(-0.19, -0.045, 0.22), // IR Left 2  (A1) — left face
    laser_right_front: new THREE.Vector3( 0.19,  0.045, 0.22), // IR Right 1 (A2) — right face
    laser_right_back:  new THREE.Vector3( 0.19, -0.045, 0.22), // IR Right 2 (A9) — right face
    laser_back_left:   new THREE.Vector3(-0.09, -0.175, 0.22), // IR Back 2  (A11) — back-left
    laser_back_right:  new THREE.Vector3( 0.09, -0.175, 0.22), // IR Back 1  (A10) — back-right
    ultra_front_left:  new THREE.Vector3(-0.095, 0.165, 0.18), // Ultrasonic FL (D22/D23)
    ultra_front_right: new THREE.Vector3( 0.095, 0.165, 0.18), // Ultrasonic FR (D24/D25)
  };

  Object.entries(labelPositions).forEach(([key, pos]) => {
    const label = createSensorLabel('---', pos);
    sensorLabels!.set(key, label);
    scene!.scene.add(label);
  });

  initHitPoints();
  setupClickToMove(scene);

  recorder = new SessionRecorder();
  playback = new PlaybackEngine();

  // Subscribe to state changes — update 3D model
  unsubscribe = onTwinStateChange((state) => {
    if (!robotGroup || !robotParts) return;

    // Position (mm -> m)
    robotGroup.position.set(
      state.position.x * 0.001,
      state.position.y * 0.001,
      0
    );

    // Heading
    robotGroup.rotation.z = -(state.heading * Math.PI) / 180;

    // Gripper
    updateGripper(state.gripperOpen, robotParts);

    // Tilt servo
    updateTiltServo(state.tiltAngle, robotParts);

    // Lifter
    updateLifter(state.lifterHeight, robotParts);

    // Line sensors — use actual line sensor data
    updateLineSensors(
      {
        line_left: state.sensors.line_left,
        line_center: state.sensors.line_center,
        line_right: state.sensors.line_right,
      },
      robotParts
    );

    // Laser sensor arcs
    if (laserArcs && hitPoints) {
      updateSensorArcs(
        {
          laser_left_front: state.sensors.laser_left_front,
          laser_left_back: state.sensors.laser_left_back,
          laser_right_front: state.sensors.laser_right_front,
          laser_right_back: state.sensors.laser_right_back,
          laser_back_left: state.sensors.laser_back_left,
          laser_back_right: state.sensors.laser_back_right,
        },
        laserArcs,
        1500,
        hitPoints
      );
    }

    // Ultrasonic sensor arcs
    if (ultraCones && hitPoints) {
      updateSensorArcs(
        {
          ultra_front_left: state.sensors.ultra_front_left,
          ultra_front_right: state.sensors.ultra_front_right,
        },
        ultraCones,
        4000,
        hitPoints
      );
    }

    // TF-Luna — use its own distance sensor
    if (tfLuna) {
      const tfDist = (state.sensors.tf_luna_distance || 1500) * 0.001;
      const headingRad = (state.heading * Math.PI) / 180;
      tfLuna.hitPoint.position.set(
        robotGroup.position.x + Math.cos(headingRad) * tfDist,
        robotGroup.position.y + Math.sin(headingRad) * tfDist,
        0
      );
    }

    // Sensor labels
    if (sensorLabels) {
      const allSensors: Record<string, number> = {
        laser_left_front: state.sensors.laser_left_front,
        laser_left_back: state.sensors.laser_left_back,
        laser_right_front: state.sensors.laser_right_front,
        laser_right_back: state.sensors.laser_right_back,
        laser_back_left: state.sensors.laser_back_left,
        laser_back_right: state.sensors.laser_back_right,
        ultra_front_left: state.sensors.ultra_front_left,
        ultra_front_right: state.sensors.ultra_front_right,
      };
      Object.entries(allSensors).forEach(([key, dist]) => {
        const label = sensorLabels!.get(key);
        if (label) {
          const maxRange = key.startsWith('ultra') ? 4000 : 1500;
          updateSensorLabel(label, dist, maxRange);
        }
      });
    }

    // Camera frustum (AI mode only)
    if (cameraFrustum) {
      cameraFrustum.visible = state.mode === 'ai';
    }

    // Wheel rotation — only update in real mode (sim mode handles this in the sim loop)
    if (simMode === 'real') {
      updateWheelRotation(1, robotParts);
    }
  });

  // Initial data fetch
  refreshTwinData();

  // Auto-start simulation so the twin is immediately drivable
  startSimulation();

  // Start real-mode polling (simulation overrides position, but polling keeps sensors live)
  startRealPolling();

  scene.animate(() => {});
}

export function destroyDigitalTwin() {
  // Stop simulation
  if (simInterval) {
    clearInterval(simInterval);
    simInterval = null;
  }
  stopRealPolling();
  physicsEngine.stop();

  // Stop playback
  playback?.stop();
  playback = null;
  recorder = null;

  // Unsubscribe twin state
  if (unsubscribe) {
    unsubscribe();
    unsubscribe = null;
  }

  // Unsubscribe WorldState
  WorldState.unsubscribe('digital-twin');

  // Dispose occupancy grid
  occGrid?.dispose();
  occGrid = null;

  // Dispose scene (removes canvas from DOM)
  if (scene) {
    scene.dispose();
    scene = null;
  }

  // Remove obstacle meshes left on the scene
  robotGroup = null;
  robotParts = null;
  laserArcs = null;
  ultraCones = null;
  hitPoints = null;
  tfLuna = null;
  cameraFrustum = null;
  sensorLabels = null;
  currentPathGroup = null;
  replayMarker = null;
  containerEl = null;
  targetMarker = null;
}

// --- Polling ---

function startRealPolling() {
  stopRealPolling();
  realPollInterval = setInterval(() => {
    if (simMode === 'real') {
      refreshTwinData();
    }
  }, 200);
}

function stopRealPolling() {
  if (realPollInterval) {
    clearInterval(realPollInterval);
    realPollInterval = null;
  }
}

export function refreshTwinData(): Promise<void> {
  const pos = fetchRobotPosition().then((pos) => {
    if (!pos.success) return;
    updateTwinState({
      position: {
        x: pos.position.x, y: pos.position.y, z: pos.position.z,
        roll: pos.orientation.roll, pitch: pos.orientation.pitch, yaw: pos.orientation.yaw,
      },
      heading: pos.orientation.yaw,
    });
  }).catch(() => {});

  const sen = fetchSensors().then((sensors) => {
    updateTwinState({
      sensors: {
        laser_left_front: sensors.laser_left_front,
        laser_left_back: sensors.laser_left_back,
        laser_right_front: sensors.laser_right_front,
        laser_right_back: sensors.laser_right_back,
        laser_back_left: sensors.laser_back_left,
        laser_back_right: sensors.laser_back_right,
        ultra_front_left: sensors.ultra_front_left,
        ultra_front_right: sensors.ultra_front_right,
        line_left: sensors.line_left,
        line_center: sensors.line_center,
        line_right: sensors.line_right,
        tf_luna_distance: sensors.tf_luna_distance,
      },
    });
  }).catch(() => {});

  return Promise.all([pos, sen]).then(() => {});
}

export async function loadWaypointPath(pathId: number) {
  if (!scene) return;

  if (currentPathGroup) {
    scene.scene.remove(currentPathGroup);
    currentPathGroup = null;
  }

  try {
    const data = await fetchPath(pathId);
    if (data.waypoints && data.waypoints.length > 0) {
      const waypoints = data.waypoints.map((wp) => ({
        x: wp.x,
        y: wp.y,
        z: 0,
        heading: wp.heading,
        order: wp.order,
        actions: wp.actions as Record<string, unknown> | undefined,
      }));

      currentPathGroup = createPathLine(waypoints);
      scene.scene.add(currentPathGroup);
    }
  } catch {
    // Path not loaded
  }
}

export function clearWaypointPath() {
  if (scene && currentPathGroup) {
    scene.scene.remove(currentPathGroup);
    currentPathGroup = null;
  }
}

export function showReplayMarker(x: number, y: number, heading: number) {
  if (!scene) return;

  if (replayMarker) {
    scene.scene.remove(replayMarker);
  }

  replayMarker = createReplayMarker({ x, y }, heading);
  scene.scene.add(replayMarker);
}

export function hideReplayMarker() {
  if (scene && replayMarker) {
    scene.scene.remove(replayMarker);
    replayMarker = null;
  }
}

// --- Click-to-move ---

function setupClickToMove(twinScene: TwinScene) {
  const raycaster = new THREE.Raycaster();
  const mouse = new THREE.Vector2();
  const groundPlane = new THREE.Plane(new THREE.Vector3(0, 0, 1), 0);

  twinScene.renderer.domElement.addEventListener('click', (event) => {
    if (event.button !== 0) return;

    const rect = twinScene.renderer.domElement.getBoundingClientRect();
    mouse.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
    mouse.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;

    raycaster.setFromCamera(mouse, twinScene.camera);
    const intersection = new THREE.Vector3();
    raycaster.ray.intersectPlane(groundPlane, intersection);

    if (intersection) {
      showTargetMarker(intersection.x, intersection.y);

      if (simMode === 'simulation') {
        // RRT* path planning — coordinates in mm for planner, Three.js positions are m
        const result = planAndFollow(intersection.x * 1000, intersection.y * 1000);
        if (result) {
          console.log(`[RRT] path planned: ${result.path.length} waypoints, reached=${result.reached}`);
        }
      } else {
        sendNavigationGoal(intersection.x, intersection.y).catch(() => {});
      }
    }
  });

  twinScene.renderer.domElement.addEventListener('contextmenu', (event) => {
    event.preventDefault();

    const rect = twinScene.renderer.domElement.getBoundingClientRect();
    mouse.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
    mouse.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;

    raycaster.setFromCamera(mouse, twinScene.camera);
    const intersection = new THREE.Vector3();
    raycaster.ray.intersectPlane(groundPlane, intersection);

    if (intersection) {
      showTargetMarker(intersection.x, intersection.y);
    }
  });
}

function showTargetMarker(x: number, y: number) {
  if (!scene) return;

  if (targetMarker) {
    scene.scene.remove(targetMarker);
  }

  const geo = new THREE.RingGeometry(0.04, 0.06, 24);
  const mat = new THREE.MeshStandardMaterial({
    color: 0xf1c21b,
    emissive: 0xf1c21b,
    emissiveIntensity: 0.8,
    transparent: true,
    opacity: 0.8,
    side: THREE.DoubleSide,
  });
  targetMarker = new THREE.Mesh(geo, mat);
  targetMarker.rotation.x = -Math.PI / 2;
  targetMarker.position.set(x, y, 0.005);
  scene.scene.add(targetMarker);

  setTimeout(() => {
    if (scene && targetMarker) {
      scene.scene.remove(targetMarker);
      targetMarker = null;
    }
  }, 3000);
}

// --- Simulation Mode ---

let physicsEngine = new PhysicsEngine();
let simPos = { x: 0, y: 0, heading: 0 };
let lastBackendCorrection = 0;

function simStep(dt: number) {
  const s = physicsEngine.step(dt);
  simPos.x = s.x;
  simPos.y = s.y;
  simPos.heading = s.heading;
}

export function startSimulation(presetName?: string) {
  if (!scene) return;

  simMode = 'simulation';
  stopRealPolling();

  // Remove old obstacle meshes
  const oldMeshes: THREE.Mesh[] = [];
  scene.scene.children.forEach((child) => {
    if (child.userData.isObstacle) oldMeshes.push(child as THREE.Mesh);
  });
  oldMeshes.forEach((m) => {
    scene!.scene.remove(m);
    m.geometry.dispose();
    (m.material as THREE.Material).dispose();
  });

  let scenario: Scenario | null = null;
  if (presetName) {
    const preset = PRESETS.find((p) => p.name === presetName);
    if (preset) scenario = preset.scenario;
  }

  let physObstacles: Obstacle[] = [];
  if (scenario) {
    physObstacles = scenarioToObstacles(scenario);
    const backendObs = physObstacles.map((obs) => ({
      x: obs.x * 0.001, y: obs.y * 0.001, w: obs.width * 0.001, h: obs.depth * 0.001,
    }));
    syncSimObstacles(backendObs).catch(() => {});
    physObstacles.forEach((obs) => {
      const geo = new THREE.BoxGeometry(obs.width * 0.001, obs.depth * 0.001, obs.height * 0.001);
      const mat = new THREE.MeshStandardMaterial({
        color: obs.color ?? 0x8b4513,
        roughness: 0.7,
        transparent: true,
        opacity: 0.8,
      });
      const mesh = new THREE.Mesh(geo, mat);
      mesh.position.set(obs.x * 0.001, obs.y * 0.001, (obs.height * 0.001) / 2);
      mesh.castShadow = true;
      mesh.receiveShadow = true;
      mesh.userData.isObstacle = true;
      scene!.scene.add(mesh);
    });
  } else {
    // No preset — sync BackendSim's current obstacles to scene
    fetchSimObstacles().then((res) => {
      (res.obstacles || []).forEach((o: { x: number; y: number; w: number; h: number }) => {
        const geo = new THREE.BoxGeometry(o.w, 0.3, o.h);
        const mat = new THREE.MeshStandardMaterial({
          color: 0x8b4513,
          roughness: 0.7,
          transparent: true,
          opacity: 0.8,
        });
        const mesh = new THREE.Mesh(geo, mat);
        mesh.position.set(o.x, o.y, o.h / 2);
        mesh.castShadow = true;
        mesh.receiveShadow = true;
        mesh.userData.isObstacle = true;
        scene!.scene.add(mesh);
      });
    }).catch(() => {});
  }

  if (simInterval) clearInterval(simInterval);

  // Reset physics engine with obstacles
  const current = getTwinState();
  physicsEngine = new PhysicsEngine();
  physicsEngine.setState({
    x: current.position.x,
    y: current.position.y,
    heading: current.heading,
    vx: 0, vy: 0, omega: 0,
  });
  physicsEngine.setObstacles(physObstacles);
  simPos = { x: current.position.x, y: current.position.y, heading: current.heading };

  // Store obstacles for RRT planning
  rrtObstacles = physObstacles;
  clearRRTVisualization();
  lastBackendCorrection = Date.now();

  simInterval = setInterval(() => {
    if (!robotGroup) return;

    simStep(0.033);
    followStep();

    updateTwinState({
      position: { x: simPos.x, y: simPos.y, z: 0, roll: 0, pitch: 0, yaw: simPos.heading },
      heading: simPos.heading,
    });

    if (robotParts) {
      updateWheelRotation(1, robotParts);
    }

    const now = Date.now();
    if (now - lastBackendCorrection > 200) {
      lastBackendCorrection = now;
    }

    // Update twin state with Arduino Mega sensor readings
    if (lastVffResult) {
      const rd = lastVffResult.sensorReadings;
      updateTwinState({
        sensors: {
          laser_left_front: rd[0] ?? 1500,
          laser_left_back: rd[1] ?? 1500,
          laser_right_front: rd[2] ?? 1500,
          laser_right_back: rd[3] ?? 1500,
          laser_back_left: rd[4] ?? 1500,
          laser_back_right: rd[5] ?? 1500,
          ultra_front_left: rd[6] ?? 5000,
          ultra_front_right: rd[7] ?? 5000,
          line_left: 0,
          line_center: 0,
          line_right: 0,
          tf_luna_distance: 0,
        },
      });
    }

    const state = getTwinState();
    WorldState.update('sim', {
      robotPosition: { x: state.position.x, y: state.position.y, heading: state.heading },
      robotPosition3D: { x: state.position.x, y: state.position.y, z: 0, roll: 0, pitch: 0, yaw: state.heading },
      sensors: state.sensors,
    });

    if (recorder?.isRecording()) {
      recorder.addFrame(state, state.sensors);
    }
  }, 33);
}

export function stopSimulation() {
  simMode = 'real';
  followActive = false;
  physicsEngine.stop();
  physicsEngine = new PhysicsEngine();
  simPos = { x: 0, y: 0, heading: 0 };
  clearRRTVisualization();
  rrtObstacles = [];
  sendContinuousCommand('s').catch(() => {});
  if (simInterval) { clearInterval(simInterval); simInterval = null; }
  startRealPolling();
  refreshTwinData();
}

export function showOccupancyGrid(visible: boolean) {
  occGrid?.setVisible(visible);
}

export function syncMapObstaclesToTwin(obstacles: Array<{ x: number; y: number; width: number; height: number; depth: number }>) {
  if (!scene) return;
  const oldMeshes: THREE.Mesh[] = [];
  scene.scene.children.forEach((child) => {
    if (child.userData.isMapObstacle) oldMeshes.push(child as THREE.Mesh);
  });
  oldMeshes.forEach((m) => {
    scene!.scene.remove(m);
    m.geometry.dispose();
    (m.material as THREE.Material).dispose();
  });

  obstacles.forEach((obs) => {
    const geo = new THREE.BoxGeometry(obs.width * 0.001, obs.height * 0.001, obs.depth * 0.001);
    const mat = new THREE.MeshStandardMaterial({
      color: 0x6666ff,
      roughness: 0.6,
      transparent: true,
      opacity: 0.6,
    });
    const mesh = new THREE.Mesh(geo, mat);
    mesh.position.set(obs.x * 0.001, obs.y * 0.001, (obs.depth * 0.001) / 2);
    mesh.userData.isMapObstacle = true;
    scene!.scene.add(mesh);
  });

  // Sync to BackendSim
  const backendObs = obstacles.map((o) => ({
    x: o.x * 0.001, y: o.y * 0.001, w: o.width * 0.001, h: o.depth * 0.001,
  }));
  syncSimObstacles(backendObs).catch(() => {});
}

export function getSimulationMode(): 'real' | 'simulation' {
  return simMode;
}

export function getSimulationPresets() {
  return PRESETS.map((p) => ({ name: p.name, description: p.description }));
}

const MAX_SPEED = 300;      // mm/s
const MAX_ROT_SPEED = 150;  // deg/s

function mapVelToCommand(vx: number, vy: number, omega: number): string {
  if (Math.abs(omega) > 10) return omega > 0 ? 't' : 'y';
  if (Math.abs(vy) > 10) return vy > 0 ? 'f' : 'b';
  if (Math.abs(vx) > 10) return vx > 0 ? 'q' : 'e';
  return 's';
}

export function commandSimulation(vx: number, vy: number, omega: number) {
  if (simMode === 'simulation') {
    physicsEngine.command(vx, vy, omega);
    const cmd = mapVelToCommand(vx, vy, omega);
    sendContinuousCommand(cmd).catch(() => {});
  }
}

export function stopSimulationRobot() {
  followActive = false;
  physicsEngine.stop();
  sendContinuousCommand('s').catch(() => {});
}

// --- Autonomous Waypoint Following ---

const DEG2RAD_FOLLOW = Math.PI / 180;
const FOLLOW_REACH_DIST = 400; // mm

let lastVffResult: VffResult | null = null;
let followActive = false;
let followWaypoints: Array<{ x: number; y: number }> = [];
let followIndex = 0;

// Stuck detection
let stuckFrameCount = 0;
let lastFollowPos = { x: 0, y: 0 };
const STUCK_THRESHOLD = 15;  // frames without movement = stuck
const STUCK_RECOVERY_FRAMES = 30;

export function getFollowStatus() {
  return { active: followActive, index: followIndex, total: followWaypoints.length };
}

export async function startPathFollowing(pathId: number) {
  try {
    const data = await fetchPath(pathId);
    if (!data.waypoints || data.waypoints.length < 2) return;
    followWaypoints = data.waypoints
      .sort((a, b) => a.order - b.order)
      .map((wp) => ({ x: wp.x * 1000, y: wp.y * 1000 }));
    followIndex = 0;
    followActive = true;
  } catch {}
}

export function stopPathFollowing() {
  followActive = false;
  followWaypoints = [];
  followIndex = 0;
  physicsEngine.stop();
  clearRRTVisualization();
  sendContinuousCommand('s').catch(() => {});
}

function followStep() {
  if (!followActive || followIndex >= followWaypoints.length) {
    if (followActive && followIndex >= followWaypoints.length) {
      followActive = false;
      physicsEngine.stop();
      stuckFrameCount = 0;
      sendContinuousCommand('s').catch(() => {});
    }
    return;
  }

  const target = followWaypoints[followIndex];
  const dx = target.x - simPos.x;
  const dy = target.y - simPos.y;
  const dist = Math.sqrt(dx * dx + dy * dy);

  if (dist < FOLLOW_REACH_DIST) {
    followIndex++;
    stuckFrameCount = 0;
    return;
  }

  // ── Stuck detection ──
  const movedDist = Math.sqrt(
    (simPos.x - lastFollowPos.x) ** 2 +
    (simPos.y - lastFollowPos.y) ** 2
  );
  lastFollowPos = { x: simPos.x, y: simPos.y };

  if (movedDist < 3) {
    stuckFrameCount++;
  } else {
    stuckFrameCount = Math.max(0, stuckFrameCount - 2);
  }

  // ── Stuck recovery ──
  if (stuckFrameCount > STUCK_THRESHOLD) {
    // Phase 1 (frames 16-40): wall-slide — project desired velocity perpendicular to obstacle
    if (stuckFrameCount < STUCK_THRESHOLD + 40) {
      const hRad = simPos.heading * DEG2RAD_FOLLOW;
      const cosH = Math.cos(hRad);
      const sinH = Math.sin(hRad);
      // Desired velocity in world frame
      const wDx = dx / dist;
      const wDy = dy / dist;
      // Try sliding: rotate desired direction by ±90° (perpendicular to obstacle)
      const slideDir = (stuckFrameCount % 40 < 20) ? 1 : -1;
      const slideWx = -wDy * slideDir;
      const slideWy = wDx * slideDir;
      // Convert back to robot frame
      const slideVx = slideWx * cosH - slideWy * sinH;
      const slideVy = slideWx * sinH + slideWy * cosH;
      const slideSpeed = 120;
      physicsEngine.command(slideVx * slideSpeed, slideVy * slideSpeed, 0);
      sendContinuousCommand('f').catch(() => {});
      return;
    }
    // Phase 2 (frames 40-60): try to move forward at reduced speed
    if (stuckFrameCount < STUCK_THRESHOLD + 60) {
      physicsEngine.command(0, 80, 0);
      sendContinuousCommand('f').catch(() => {});
      return;
    }
    // Phase 3 (frames 60+): skip waypoint
    console.warn(`[Follow] stuck at wp ${followIndex}/${followWaypoints.length}, skipping`);
    followIndex++;
    stuckFrameCount = 0;
    return;
  }

  // Desired direction toward target in robot frame
  const hRad = simPos.heading * DEG2RAD_FOLLOW;
  const cosH = Math.cos(hRad);
  const sinH = Math.sin(hRad);
  const idealVx = (dx * cosH - dy * sinH) / dist;
  const idealVy = (dx * sinH + dy * cosH) / dist;
  const speed = Math.min(MAX_SPEED * 0.8, Math.max(100, dist * 0.3));
  const cmdVx = idealVx * speed;
  const cmdVy = idealVy * speed;

  // ── Arduino Mega simulation: VFF + perimeter safety ──
  const result = processArduinoMega(
    cmdVx, cmdVy, 0,
    simPos.x, simPos.y,
    simPos.heading,
    (x: number, y: number) => physicsEngine.wouldCollide(x, y),
  );

  lastVffResult = result;

  if (result.braking) {
    // Wall-slide: project desired velocity along obstacle surface
    // Try ±45° rotations of desired direction to find clear path
    const hRadW = simPos.heading * DEG2RAD_FOLLOW;
    const cosHW = Math.cos(hRadW);
    const sinHW = Math.sin(hRadW);
    const wDx = dx / dist;
    const wDy = dy / dist;

    for (const angle of [0.4, -0.4, 0.8, -0.8, 1.2, -1.2]) {
      const rotX = wDx * Math.cos(angle) - wDy * Math.sin(angle);
      const rotY = wDx * Math.sin(angle) + wDy * Math.cos(angle);
      const testX = simPos.x + rotX * 100;
      const testY = simPos.y + rotY * 100;
      if (!physicsEngine.wouldCollide(testX, testY)) {
        const rvx = rotX * cosHW + rotY * sinHW;
        const rvy = -rotX * sinHW + rotY * cosHW;
        const slowSpeed = speed * 0.5;
        physicsEngine.command(rvx * slowSpeed, rvy * slowSpeed, 0);
        sendContinuousCommand('f').catch(() => {});
        return;
      }
    }
    // All directions blocked — stop briefly
    physicsEngine.command(0, 0, 0);
    sendContinuousCommand('s').catch(() => {});
    return;
  }

  // Gentle rotate toward VFF-modified movement direction
  const wVx = result.vx * cosH + result.vy * sinH;
  const wVy = -result.vx * sinH + result.vy * cosH;
  let headingDelta = 0;
  if (Math.abs(result.vx) > 1 || Math.abs(result.vy) > 1) {
    const moveWorldAngle = (Math.atan2(wVx, wVy) * 180 / Math.PI + 360) % 360;
    headingDelta = moveWorldAngle - simPos.heading;
    if (headingDelta > 180) headingDelta -= 360;
    if (headingDelta < -180) headingDelta += 360;
  }

  physicsEngine.command(result.vx, result.vy, Math.sign(headingDelta) * Math.min(50, Math.abs(headingDelta) * 1.5));
  sendContinuousCommand('f').catch(() => {});
}

// ── RRT* Path Planning ────────────────────────────────────────────────────

/** Plan an RRT* path from current position to a goal (mm) and start following it. */
export function planAndFollow(goalX: number, goalY: number): RRTResult | null {
  if (simMode !== 'simulation') return null;

  const start: Point = { x: simPos.x, y: simPos.y };
  const goal: Point = { x: goalX, y: goalY };

  rrtResult = planRRT(start, goal, rrtObstacles);

  console.log(`[RRT] planned: ${rrtResult.path.length} waypoints, reached=${rrtResult.reached}, tree=${rrtResult.tree.length} nodes`);

  if (rrtResult.path.length < 2) return rrtResult;

  // Convert to follow waypoints (mm) and start following
    followWaypoints = rrtResult.path.map((p) => ({ x: p.x, y: p.y }));
    followIndex = 0;
    followActive = true;
    stuckFrameCount = 0;
    lastFollowPos = { x: simPos.x, y: simPos.y };

  drawRRTVisualization(rrtResult);

  return rrtResult;
}

/** Plan an RRT* path without starting to follow (just visualise or inspect). */
export function planRRTPath(goalX: number, goalY: number): RRTResult | null {
  const start: Point = { x: simPos.x, y: simPos.y };
  rrtResult = planRRT(start, { x: goalX, y: goalY }, rrtObstacles);
  drawRRTVisualization(rrtResult);
  return rrtResult;
}

function drawRRTVisualization(result: RRTResult) {
  clearRRTVisualization();
  if (!scene || result.tree.length === 0) return;

  // ── Draw tree edges (faint grey) ──
  rrtTreeGroup = new THREE.Group();
  const treeMat = new THREE.LineBasicMaterial({ color: 0x888888, transparent: true, opacity: 0.25 });
  const treeGeo = new THREE.BufferGeometry();

  const verts: number[] = [];
  for (const node of result.tree) {
    if (node.parent !== null) {
      const p = result.tree[node.parent];
      verts.push(p.x * 0.001, p.y * 0.001, 0.02,
                  node.x * 0.001, node.y * 0.001, 0.02);
    }
  }
  treeGeo.setAttribute('position', new THREE.Float32BufferAttribute(verts, 3));
  const treeLine = new THREE.LineSegments(treeGeo, treeMat);
  rrtTreeGroup.add(treeLine);

  // ── Draw final path (bright cyan) ──
  if (result.path.length >= 2) {
    const pathMat = new THREE.LineBasicMaterial({ color: 0x00e5ff, linewidth: 2 });
    const pathGeo = new THREE.BufferGeometry();
    const pathVerts: number[] = [];
    for (const p of result.path) {
      pathVerts.push(p.x * 0.001, p.y * 0.001, 0.03);
    }
    pathGeo.setAttribute('position', new THREE.Float32BufferAttribute(pathVerts, 3));
    const pathLine = new THREE.Line(pathGeo, pathMat);
    rrtTreeGroup.add(pathLine);

    // Draw waypoint spheres along path
    const sphereGeo = new THREE.SphereGeometry(0.04, 8, 8);
    const sphereMat = new THREE.MeshStandardMaterial({ color: 0x00e5ff, emissive: 0x00e5ff, emissiveIntensity: 0.5 });
    for (let i = 1; i < result.path.length - 1; i++) {
      const sphere = new THREE.Mesh(sphereGeo, sphereMat);
      sphere.position.set(result.path[i].x * 0.001, result.path[i].y * 0.001, 0.04);
      rrtTreeGroup.add(sphere);
    }
  }

  scene.scene.add(rrtTreeGroup);
}

function clearRRTVisualization() {
  if (rrtTreeGroup && scene) {
    scene.scene.remove(rrtTreeGroup);
    rrtTreeGroup.traverse((child: any) => {
      if (child.geometry) child.geometry.dispose();
      if (child.material) {
        if (Array.isArray(child.material)) child.material.forEach((m: any) => m.dispose());
        else child.material.dispose();
      }
    });
    rrtTreeGroup = null;
  }
  rrtResult = null;
}

/** Expose current obstacles for external RRT calls. */
export function getRRTOstacles(): Obstacle[] { return rrtObstacles; }

/** Expose RRT result for UI status display. */
export function getRRTResult(): RRTResult | null { return rrtResult; }

// --- Recording ---

export function startRecording(name?: string): string {
  if (!recorder) return '';
  return recorder.start(name);
}

export function stopRecording(): Recording | null {
  if (!recorder) return null;
  const recording = recorder.stop();
  if (recording) {
    saveRecording(recording);
  }
  return recording;
}

export function isRecording(): boolean {
  return recorder?.isRecording() ?? false;
}

export function getRecordingFrameCount(): number {
  return recorder?.getFrameCount() ?? 0;
}

// --- Playback ---

export function loadPlayback(recording: Recording) {
  if (!playback) return;
  playback.load(recording);
}

export function startPlayback(
  onFrame: (frame: any, progress: PlaybackProgress) => void,
  onComplete?: () => void
) {
  if (!playback) return;
  playback.play(onFrame, onComplete);
}

export function pausePlayback() {
  playback?.pause();
}

export function resumePlayback() {
  playback?.resume();
}

export function stopPlayback() {
  playback?.stop();
  if (robotGroup) {
    robotGroup.position.set(0, 0, 0);
    robotGroup.rotation.z = 0;
  }
}

export function seekPlayback(timeMs: number) {
  playback?.seek(timeMs);
}

export function seekPlaybackPercent(pct: number) {
  playback?.seekPercent(pct);
}

export function setPlaybackSpeed(speed: number) {
  playback?.setSpeed(speed);
}

export function getPlaybackProgress(): PlaybackProgress | null {
  return playback?.getProgress() ?? null;
}

export function getPlaybackState() {
  return playback?.getState() ?? 'idle';
}
