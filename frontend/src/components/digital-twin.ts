// Digital Twin — main 3D scene controller

import { createScene, type TwinScene } from '../engine/scene';
import { createRobotModel, updateGripper, updateTiltServo, updateLifter, updateLineSensors, updateWheelRotation } from '../engine/robot-model';
import { createEnvironment } from '../engine/environment';
import { createLaserArcs, createUltrasonicCones, createTfLunaRay, updateSensorArcs, addLaserArcsToScene, addUltrasonicConesToScene } from '../engine/sensor-viz';
import { createPathLine, createReplayMarker } from '../engine/waypoint-viz';
import { createCameraFrustum, createSensorLabel, updateSensorLabel } from '../engine/sensor-labels';
import { PhysicsEngine } from '../engine/physics';
import { MockSensorGenerator } from '../engine/mock-sensors';
import { scenarioToObstacles, PRESETS, type Scenario } from '../engine/scenario';
import { SessionRecorder, saveRecording, type Recording } from '../engine/recording';
import { PlaybackEngine, type PlaybackProgress } from '../engine/playback';
import { onTwinStateChange, getTwinState, updateTwinState } from '../state/twin-state';
import { fetchRobotPosition, fetchSensors, fetchPath, moveRobot } from '../api';
import type { RobotModelParts } from '../types/twin';
import type { SensorArcMesh } from '../engine/sensor-viz';
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
let physics: PhysicsEngine | null = null;
let mockSensors: MockSensorGenerator | null = null;
let simMode: 'real' | 'simulation' = 'real';
let simInterval: ReturnType<typeof setInterval> | null = null;
let realPollInterval: ReturnType<typeof setInterval> | null = null;
let recorder: SessionRecorder | null = null;
let playback: PlaybackEngine | null = null;

let targetMarker: THREE.Mesh | null = null;
let moveTarget: { x: number; y: number } | null = null;
let moveLoopId: ReturnType<typeof setInterval> | null = null;

// Keyboard drive state
const keyState: Record<string, boolean> = {};
let keyDriveInterval: ReturnType<typeof setInterval> | null = null;

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

  physics = new PhysicsEngine();
  mockSensors = new MockSensorGenerator();
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
    robotGroup.rotation.z = (state.heading * Math.PI) / 180;

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

    // Wheel rotation — use velocity magnitude
    const vel = Math.sqrt(
      (state.position.x !== 0 ? 1 : 0) ** 2 +
      (state.position.y !== 0 ? 1 : 0) ** 2
    );
    // In sim mode, read velocity from physics; in real mode, approximate from position changes
    updateWheelRotation(vel > 0 ? 1 : 0, robotParts);
  });

  // Initial data fetch
  refreshTwinData();

  // Auto-start simulation so the twin is immediately drivable
  startSimulation();
  setupKeyboardDrive();

  // Start real-mode polling (simulation overrides position, but polling keeps sensors live)
  startRealPolling();

  scene.animate(() => {});
}

export function destroyDigitalTwin() {
  stopRealPolling();
  stopMoveLoop();
  stopKeyboardDrive();

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
  moveTarget = null;
  targetMarker = null;
}

// --- Real-mode polling ---

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

export function refreshTwinData() {
  fetchRobotPosition()
    .then((pos) => {
      if (pos.success) {
        updateTwinState({
          position: {
            x: pos.position.x,
            y: pos.position.y,
            z: pos.position.z,
            roll: pos.orientation.roll,
            pitch: pos.orientation.pitch,
            yaw: pos.orientation.yaw,
          },
          heading: pos.orientation.yaw,
        });
      }
    })
    .catch(() => {});

  fetchSensors()
    .then((sensors) => {
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
    })
    .catch(() => {});
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
      const xMm = Math.round(intersection.x * 1000);
      const yMm = Math.round(intersection.y * 1000);

      showTargetMarker(intersection.x, intersection.y);

      if (simMode === 'simulation') {
        // Simulation: use physics engine to move toward target
        startMoveToTarget(xMm, yMm);
      } else {
        // Real mode: send turn + move commands
        const state = getTwinState();
        const dx = xMm - state.position.x;
        const dy = yMm - state.position.y;
        const dist = Math.sqrt(dx * dx + dy * dy);
        if (dist < 50) return; // too close

        const targetHeading = (Math.atan2(dx, dy) * 180) / Math.PI;
        const headingDiff = ((targetHeading - state.heading + 540) % 360) - 180;

        // Turn toward target, then move forward
        if (Math.abs(headingDiff) > 5) {
          const turnCmd = headingDiff > 0 ? 'tl' : 'tr';
          moveRobot(turnCmd, Math.min(Math.abs(headingDiff), 90)).catch(() => {});
        }
        // Move forward for calculated duration (dist / speed)
        const durationMs = Math.round((dist / 200) * 1000);
        moveRobot('forward', 200, durationMs).catch(() => {});
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

function startMoveToTarget(targetX: number, targetY: number) {
  stopMoveLoop();
  moveTarget = { x: targetX, y: targetY };

  moveLoopId = setInterval(() => {
    if (!physics || !moveTarget) {
      stopMoveLoop();
      return;
    }

    const state = getTwinState();
    const dx = moveTarget.x - state.position.x;
    const dy = moveTarget.y - state.position.y;
    const dist = Math.sqrt(dx * dx + dy * dy);

    if (dist < 30) {
      // Reached target
      physics.command(0, 0, 0);
      stopMoveLoop();
      return;
    }

    // Move toward target at 200 mm/s
    const speed = 200;
    const vx = (dx / dist) * speed;
    const vy = (dy / dist) * speed;
    physics.command(vx, vy, 0);
  }, 33);
}

function stopMoveLoop() {
  if (moveLoopId) {
    clearInterval(moveLoopId);
    moveLoopId = null;
  }
  moveTarget = null;
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

// --- Keyboard Drive ---

const DRIVE_KEYS = ['w','s','a','d','q','e','ArrowUp','ArrowDown','ArrowLeft','ArrowRight',' '];

function setupKeyboardDrive() {
  const onKeyDown = (e: KeyboardEvent) => {
    if (!DRIVE_KEYS.includes(e.key)) return;
    e.preventDefault();
    keyState[e.key] = true;
  };
  const onKeyUp = (e: KeyboardEvent) => {
    delete keyState[e.key];
  };
  document.addEventListener('keydown', onKeyDown);
  document.addEventListener('keyup', onKeyUp);

  // Poll at 20 Hz and push velocity to physics
  keyDriveInterval = setInterval(() => {
    if (simMode !== 'simulation' || !physics) return;
    let vx = 0, vy = 0, omega = 0;
    if (keyState['w'] || keyState['ArrowUp'])    vy =  300;
    if (keyState['s'] || keyState['ArrowDown'])  vy = -300;
    if (keyState['a'] || keyState['ArrowLeft'])  vx = -300;
    if (keyState['d'] || keyState['ArrowRight']) vx =  300;
    if (keyState['q']) omega =  90;
    if (keyState['e']) omega = -90;
    if (keyState[' ']) { vx = 0; vy = 0; omega = 0; }
    if (vx !== 0 || vy !== 0 || omega !== 0 || keyState[' ']) {
      physics.command(vx, vy, omega);
    }
  }, 50);
}

function stopKeyboardDrive() {
  if (keyDriveInterval) {
    clearInterval(keyDriveInterval);
    keyDriveInterval = null;
  }
  Object.keys(keyState).forEach((k) => delete keyState[k]);
}

// --- Simulation Mode ---

export function startSimulation(presetName?: string) {
  if (!scene || !physics || !mockSensors) return;

  simMode = 'simulation';
  stopRealPolling();

  let scenario: Scenario | null = null;
  if (presetName) {
    const preset = PRESETS.find((p) => p.name === presetName);
    if (preset) scenario = preset.scenario;
  }

  const obstacleMeshes: THREE.Mesh[] = [];

  if (scenario) {
    physics.setState(scenario.robotStart);

    const obstacles = scenarioToObstacles(scenario);
    physics.setObstacles(obstacles);

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

    // Create new obstacle meshes
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
      obstacleMeshes.push(mesh);
    });
  }

  // Pass obstacle meshes to mock sensor generator
  mockSensors.setObstacleMeshes(obstacleMeshes);

  // Start simulation loop
  if (simInterval) clearInterval(simInterval);
  simInterval = setInterval(() => {
    if (!physics || !mockSensors) return;

    const simState = physics.step(1 / 30);
    const sensorData = mockSensors.generate(simState, scene!.scene);

    // Update robot in scene
    if (robotGroup) {
      robotGroup.position.set(simState.x * 0.001, simState.y * 0.001, 0);
      robotGroup.rotation.z = (simState.heading * Math.PI) / 180;
    }

    // Update twin state
    updateTwinState({
      position: {
        x: simState.x,
        y: simState.y,
        z: 0,
        roll: 0,
        pitch: 0,
        yaw: simState.heading,
      },
      heading: simState.heading,
      sensors: {
        laser_left_front: sensorData.laser_left_front,
        laser_left_back: sensorData.laser_left_back,
        laser_right_front: sensorData.laser_right_front,
        laser_right_back: sensorData.laser_right_back,
        laser_back_left: sensorData.laser_back_left,
        laser_back_right: sensorData.laser_back_right,
        ultra_front_left: sensorData.ultra_front_left,
        ultra_front_right: sensorData.ultra_front_right,
        line_left: sensorData.line_left,
        line_center: sensorData.line_center,
        line_right: sensorData.line_right,
        tf_luna_distance: sensorData.ultra_front_left,
      },
    });

    // Wheel rotation — use velocity magnitude from physics
    const velMag = Math.sqrt(simState.vx ** 2 + simState.vy ** 2);
    if (robotParts) {
      updateWheelRotation(velMag > 1 ? 1 : 0, robotParts);
    }

    // Record frame if recording
    if (recorder?.isRecording()) {
      recorder.addFrame(getTwinState(), sensorData);
    }
  }, 33); // ~30 Hz
}

export function stopSimulation() {
  simMode = 'real';
  if (simInterval) {
    clearInterval(simInterval);
    simInterval = null;
  }
  physics?.stop();
  stopMoveLoop();
  startRealPolling();
  refreshTwinData();
}

export function getSimulationMode(): 'real' | 'simulation' {
  return simMode;
}

export function getSimulationPresets() {
  return PRESETS.map((p) => ({ name: p.name, description: p.description }));
}

export function commandSimulation(vx: number, vy: number, omega: number) {
  if (simMode === 'simulation' && physics) {
    physics.command(vx, vy, omega);
  }
}

export function stopSimulationRobot() {
  stopMoveLoop();
  if (physics) {
    physics.stop();
  }
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
