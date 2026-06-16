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
import type { RobotModelParts } from '../types/twin';
import type { SensorArcMesh } from '../engine/sensor-viz';
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
  if (scene) return;
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
  stopRealPolling();

  if (unsubscribe) {
    unsubscribe();
    unsubscribe = null;
  }
  if (scene) {
    scene.dispose();
    scene = null;
  }
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

      sendNavigationGoal(intersection.x, intersection.y).catch(() => {});
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

let simVel = { vx: 0, vy: 0, omega: 0 };
let simPos = { x: 0, y: 0, heading: 0 };
let lastBackendCorrection = 0;

function simStep(dt: number) {
  const { vx, vy, omega } = simVel;
  const hRad = (simPos.heading * Math.PI) / 180;
  const worldVx = vx * Math.cos(hRad) + vy * Math.sin(hRad);
  const worldVy = -vx * Math.sin(hRad) + vy * Math.cos(hRad);
  simPos.x += worldVx * dt;
  simPos.y += worldVy * dt;
  simPos.heading = (simPos.heading + omega * dt + 360) % 360;
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

  if (scenario) {
    const obstacles = scenarioToObstacles(scenario);
    const backendObs = obstacles.map((obs) => ({
      x: obs.x * 0.001, y: obs.y * 0.001, w: obs.width * 0.001, h: obs.depth * 0.001,
    }));
    syncSimObstacles(backendObs).catch(() => {});
    obstacles.forEach((obs) => {
      const geo = new THREE.BoxGeometry(obs.width * 0.001, obs.height * 0.001, obs.depth * 0.001);
      const mat = new THREE.MeshStandardMaterial({
        color: obs.color ?? 0x8b4513,
        roughness: 0.7,
        transparent: true,
        opacity: 0.8,
      });
      const mesh = new THREE.Mesh(geo, mat);
      mesh.position.set(obs.x * 0.001, obs.y * 0.001, (obs.depth * 0.001) / 2);
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

  simPos = { x: 0, y: 0, heading: 0 };
  refreshTwinData();
  lastBackendCorrection = Date.now();

  simInterval = setInterval(() => {
    if (!robotGroup) return;

    simStep(0.033);

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
      fetchSensors().then((sensors) => {
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
  simVel = { vx: 0, vy: 0, omega: 0 };
  simPos = { x: 0, y: 0, heading: 0 };
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
    simVel = { vx, vy, omega };
    const cmd = mapVelToCommand(vx, vy, omega);
    sendContinuousCommand(cmd).catch(() => {});
  }
}

export function stopSimulationRobot() {
  simVel = { vx: 0, vy: 0, omega: 0 };
  sendContinuousCommand('s').catch(() => {});
}

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
