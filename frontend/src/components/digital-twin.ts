// Digital Twin — main 3D scene controller

import { createScene, type TwinScene } from '../engine/scene';
import { createRobotModel, updateGripper, updateTiltServo, updateLifter, updateLineSensors, updateWheelRotation } from '../engine/robot-model';
import { createEnvironment } from '../engine/environment';
import { createLaserArcs, createUltrasonicCones, createTfLunaRay, updateSensorArcs, addLaserArcsToScene, addUltrasonicConesToScene } from '../engine/sensor-viz';
import { createPathLine, createReplayMarker } from '../engine/waypoint-viz';
import { createCameraFrustum, createSensorLabel, updateSensorLabel } from '../engine/sensor-labels';
import { PhysicsEngine, type KinematicState } from '../engine/physics';
import { MockSensorGenerator, type MockSensorData } from '../engine/mock-sensors';
import { scenarioToObstacles, PRESETS, type Scenario } from '../engine/scenario';
import { SessionRecorder, saveRecording, type Recording } from '../engine/recording';
import { PlaybackEngine, type PlaybackProgress } from '../engine/playback';
import { onTwinStateChange, getTwinState, updateTwinState, getWaypoints } from '../state/twin-state';
import { fetchRobotPosition, fetchSensors, fetchPath, fetchPaths, moveRobot } from '../api';
import type { RobotModelParts } from '../types/twin';
import type { SensorArcMesh } from '../engine/sensor-viz';
import type { SensorData } from '../types';

let scene: TwinScene | null = null;
let robotGroup: any = null;
let robotParts: RobotModelParts | null = null;
let laserArcs: Map<string, SensorArcMesh> | null = null;
let ultraCones: Map<string, SensorArcMesh> | null = null;
let hitPoints: Map<string, any> | null = null;
let tfLuna: { line: any; hitPoint: any } | null = null;
let unsubscribe: (() => void) | null = null;
let sensorUnsubscribe: (() => void) | null = null;
let containerEl: HTMLElement | null = null;
let cameraFrustum: any = null;
let sensorLabels: Map<string, any> | null = null;
let currentPathGroup: any = null;
let replayMarker: any = null;
let physics: PhysicsEngine | null = null;
let mockSensors: MockSensorGenerator | null = null;
let simMode: 'real' | 'simulation' = 'real';
let simInterval: ReturnType<typeof setInterval> | null = null;
let recorder: SessionRecorder | null = null;
let playback: PlaybackEngine | null = null;
let playbackProgress: PlaybackProgress | null = null;

import * as THREE from 'three';

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

  // Create scene
  scene = createScene(container);

  // Create robot model
  const model = createRobotModel();
  robotGroup = model.group;
  robotParts = model.parts;
  scene.scene.add(robotGroup);

  // Create environment
  createEnvironment(scene.scene);

  // Create sensor visualizations
  laserArcs = createLaserArcs();
  ultraCones = createUltrasonicCones();
  tfLuna = createTfLunaRay();

  addLaserArcsToScene(laserArcs, scene.scene);
  addUltrasonicConesToScene(ultraCones, scene.scene);
  scene.scene.add(tfLuna.line);
  scene.scene.add(tfLuna.hitPoint);

  // Create camera frustum
  cameraFrustum = createCameraFrustum();
  scene.scene.add(cameraFrustum);

  // Create sensor distance labels
  sensorLabels = new Map();
  const labelPositions: Record<string, THREE.Vector3> = {
    laser_left_front: new THREE.Vector3(0.18, 0.1, 0.22),
    laser_left_back: new THREE.Vector3(0.18, -0.1, 0.22),
    laser_right_front: new THREE.Vector3(-0.18, 0.1, 0.22),
    laser_right_back: new THREE.Vector3(-0.18, -0.1, 0.22),
    laser_back_left: new THREE.Vector3(0.08, -0.2, 0.22),
    laser_back_right: new THREE.Vector3(-0.08, -0.2, 0.22),
    ultra_front_left: new THREE.Vector3(0.08, 0.2, 0.18),
    ultra_front_right: new THREE.Vector3(-0.08, 0.2, 0.18),
  };

  Object.entries(labelPositions).forEach(([key, pos]) => {
    const label = createSensorLabel('---', pos);
    sensorLabels!.set(key, label);
    scene!.scene.add(label);
  });

  initHitPoints();

  // Setup click-to-move interaction
  setupClickToMove(scene);

  // Initialize simulation engine
  physics = new PhysicsEngine();
  mockSensors = new MockSensorGenerator();
  recorder = new SessionRecorder();
  playback = new PlaybackEngine();

  // Subscribe to state changes
  unsubscribe = onTwinStateChange((state) => {
    if (!robotGroup || !robotParts) return;

    // Update position
    robotGroup.position.set(
      state.position.x * 0.001, // mm to m
      state.position.y * 0.001,
      0
    );

    // Update heading
    robotGroup.rotation.z = (state.heading * Math.PI) / 180;

    // Update gripper
    updateGripper(state.gripperOpen, robotParts);

    // Update tilt servo (tilts gripper + camera + TF-Luna together)
    updateTiltServo(state.tiltAngle, robotParts);

    // Update lifter (raises/lowers gripper assembly)
    updateLifter(state.lifterHeight, robotParts);

    // Update line sensors (from actual line sensor data)
    updateLineSensors(
      {
        line_left: state.sensors.laser_left_front,
        line_center: state.sensors.laser_left_front,
        line_right: state.sensors.laser_right_front,
      },
      robotParts
    );

    // Update sensor arcs
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

    // Update TF-Luna
    if (tfLuna) {
      const tfDist = (state.sensors.ultra_front_left || 1500) * 0.001;
      const dir = 0; // forward
      tfLuna.hitPoint.position.set(
        Math.cos(dir) * tfDist,
        Math.sin(dir) * tfDist,
        0
      );
    }

    // Update sensor labels
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

    // Update camera frustum visibility (only in AI mode)
    if (cameraFrustum) {
      cameraFrustum.visible = state.mode === 'ai';
    }

    // Update wheel rotation based on movement
    updateWheelRotation(state.position.x !== 0 ? 1 : 0, robotParts);
  });

  // Initial data fetch
  refreshTwinData();

  // Animation loop — update sensor data periodically
  scene.animate(() => {
    // Periodic refresh (every ~100ms via requestAnimationFrame)
  });
}

export function destroyDigitalTwin() {
  if (unsubscribe) {
    unsubscribe();
    unsubscribe = null;
  }
  if (sensorUnsubscribe) {
    sensorUnsubscribe();
    sensorUnsubscribe = null;
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
        },
      });
    })
    .catch(() => {});
}

export async function loadWaypointPath(pathId: number) {
  if (!scene) return;

  // Remove old path
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

function setupClickToMove(twinScene: TwinScene) {
  const raycaster = new THREE.Raycaster();
  const mouse = new THREE.Vector2();
  const groundPlane = new THREE.Plane(new THREE.Vector3(0, 0, 1), 0);

  twinScene.renderer.domElement.addEventListener('click', (event) => {
    // Ignore if orbit controls are being used (right-click or middle-click)
    if (event.button !== 0) return;

    const rect = twinScene.renderer.domElement.getBoundingClientRect();
    mouse.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
    mouse.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;

    raycaster.setFromCamera(mouse, twinScene.camera);
    const intersection = new THREE.Vector3();
    raycaster.ray.intersectPlane(groundPlane, intersection);

    if (intersection) {
      // Convert Three.js meters to backend millimeters
      const xMm = Math.round(intersection.x * 1000);
      const yMm = Math.round(intersection.y * 1000);

      // Calculate heading to target
      const state = getTwinState();
      const dx = xMm - state.position.x;
      const dy = yMm - state.position.y;
      const heading = (Math.atan2(dx, dy) * 180) / Math.PI;

      // Show target marker
      showTargetMarker(intersection.x, intersection.y);

      // Send movement commands
      moveRobot('forward', 200).catch(() => {});
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

let targetMarker: THREE.Mesh | null = null;

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

  // Auto-remove after 3 seconds
  setTimeout(() => {
    if (scene && targetMarker) {
      scene.scene.remove(targetMarker);
      targetMarker = null;
    }
  }, 3000);
}

// --- Simulation Mode ---

export function startSimulation(presetName?: string) {
  if (!scene || !physics || !mockSensors) return;

  simMode = 'simulation';

  // Load preset scenario
  let scenario: Scenario | null = null;
  if (presetName) {
    const preset = PRESETS.find((p) => p.name === presetName);
    if (preset) scenario = preset.scenario;
  }

  if (scenario) {
    // Set robot start position
    physics.setState(scenario.robotStart);

    // Set obstacles
    const obstacles = scenarioToObstacles(scenario);
    physics.setObstacles(obstacles);

    // Update obstacle meshes in scene
    const oldMeshes: THREE.Mesh[] = [];
    scene.scene.children.forEach((child) => {
      if (child.userData.isObstacle) oldMeshes.push(child as THREE.Mesh);
    });
    oldMeshes.forEach((m) => {
      scene!.scene.remove(m);
      m.geometry.dispose();
      (m.material as THREE.Material).dispose();
    });

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
  }

  // Start simulation loop
  if (simInterval) clearInterval(simInterval);
  simInterval = setInterval(() => {
    if (!physics || !mockSensors) return;

    const simState = physics.step(1 / 30);
    const sensorData = mockSensors.generate(simState, scene!.scene);

    // Update robot position in scene
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
      },
    });

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
