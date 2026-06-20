import * as THREE from 'three';
import type { KinematicState } from './physics';

const MM_PER_M = 1000;
const LASER_MAX_MM = 1500;
const ULTRASOUND_MAX_MM = 4000;

interface MockSensorConfig {
  laserPositions: Array<{ name: string; x: number; y: number; angle: number }>;
  ultraPositions: Array<{ name: string; x: number; y: number; angle: number }>;
  linePositions: Array<{ x: number; y: number }>;
}

const DEFAULT_CONFIG: MockSensorConfig = {
  laserPositions: [
    { name: 'laser_left_front', x: 180, y: 100, angle: 0.4 },
    { name: 'laser_left_back', x: 180, y: -100, angle: -0.4 },
    { name: 'laser_right_front', x: -180, y: 100, angle: Math.PI - 0.4 },
    { name: 'laser_right_back', x: -180, y: -100, angle: Math.PI + 0.4 },
    { name: 'laser_back_left', x: 80, y: -200, angle: -Math.PI / 2 - 0.3 },
    { name: 'laser_back_right', x: -80, y: -200, angle: -Math.PI / 2 + 0.3 },
  ],
  ultraPositions: [
    { name: 'ultra_front_left', x: 80, y: 200, angle: 0 },
    { name: 'ultra_front_right', x: -80, y: 200, angle: 0 },
  ],
  linePositions: [
    { x: -60, y: 150 },
    { x: 0, y: 150 },
    { x: 60, y: 150 },
  ],
};

export interface MockSensorData {
  laser_left_front: number;
  laser_left_back: number;
  laser_right_front: number;
  laser_right_back: number;
  laser_back_left: number;
  laser_back_right: number;
  ultra_front_left: number;
  ultra_front_right: number;
  line_left: number;
  line_center: number;
  line_right: number;
  imu_heading: number;
  imu_pitch: number;
  imu_roll: number;
}

export interface NoiseConfig {
  laserSigma: number;
  ultraSigma: number;
  lineFlipProb: number;
  dropoutProb: number;
  headingDriftDegPerSec: number;
  ultraCrosstalkThreshold: number;
  ultraCrosstalkInfluence: number;
}

const DEFAULT_NOISE: NoiseConfig = {
  laserSigma: 15,
  ultraSigma: 30,
  lineFlipProb: 0.02,
  dropoutProb: 0.02,
  headingDriftDegPerSec: 0.5,
  ultraCrosstalkThreshold: 200,
  ultraCrosstalkInfluence: 0.15,
};

let noiseConfig: NoiseConfig = { ...DEFAULT_NOISE };
let accumulatedDrift = 0;

export function setNoiseConfig(cfg: Partial<NoiseConfig>) {
  Object.assign(noiseConfig, cfg);
}

export function resetNoiseConfig() {
  noiseConfig = { ...DEFAULT_NOISE };
  accumulatedDrift = 0;
}

function gaussianNoise(sigma: number): number {
  let u = 0, v = 0;
  while (u === 0) u = Math.random();
  while (v === 0) v = Math.random();
  return sigma * Math.sqrt(-2.0 * Math.log(u)) * Math.cos(2.0 * Math.PI * v);
}

function applyNoise(reading: number, sigma: number, maxVal: number, dropout: boolean): number {
  if (dropout && Math.random() < noiseConfig.dropoutProb) return maxVal;
  const noisy = reading + gaussianNoise(sigma);
  return Math.round(Math.max(0, Math.min(maxVal, noisy)));
}

interface ImuState {
  gyroBiasX: number;
  gyroBiasY: number;
  gyroBiasZ: number;
  biasWalkX: number;
  biasWalkY: number;
  biasWalkZ: number;
  lastHeading: number;
  velocityX: number;
  velocityY: number;
  lastTime: number;
}

function createImuState(): ImuState {
  return {
    gyroBiasX: gaussianNoise(0.5),
    gyroBiasY: gaussianNoise(0.5),
    gyroBiasZ: gaussianNoise(0.3),
    biasWalkX: 0,
    biasWalkY: 0,
    biasWalkZ: 0,
    lastHeading: 0,
    velocityX: 0,
    velocityY: 0,
    lastTime: 0,
  };
}

function simulateImu(
  state: ImuState,
  headingDeg: number,
  vx: number,
  vy: number,
  dt: number,
  baseDrift: number
): { heading: number; pitch: number; roll: number } {
  const biasInstability = 0.01;
  const angleRandomWalk = 0.05;
  const accelNoise = 2.0;

  state.biasWalkX += gaussianNoise(biasInstability) * Math.sqrt(dt);
  state.biasWalkY += gaussianNoise(biasInstability) * Math.sqrt(dt);
  state.biasWalkZ += gaussianNoise(biasInstability) * Math.sqrt(dt);

  const gyroDriftZ = state.gyroBiasZ + state.biasWalkZ + gaussianNoise(angleRandomWalk) * Math.sqrt(dt);
  const noisyHeading = headingDeg + gyroDriftZ * dt + baseDrift * dt;

  const accelX = (vx - state.velocityX) / Math.max(dt, 0.001);
  const accelY = (vy - state.velocityY) / Math.max(dt, 0.001);

  state.velocityX = vx;
  state.velocityY = vy;

  const pitchNoise = gaussianNoise(accelNoise) * 0.01;
  const rollNoise = gaussianNoise(accelNoise) * 0.01;

  const pitch = Math.atan2(accelX, 9810) * (180 / Math.PI) + pitchNoise;
  const roll = Math.atan2(accelY, 9810) * (180 / Math.PI) + rollNoise;

  state.lastHeading = headingDeg;
  state.lastTime = performance.now();

  return {
    heading: noisyHeading,
    pitch: Math.max(-90, Math.min(90, pitch)),
    roll: Math.max(-90, Math.min(90, roll)),
  };
}

function simulateIrCone(
  x: number,
  y: number,
  headingAngle: number,
  sensorAngle: number,
  maxDist: number,
  scene?: THREE.Scene,
  meshes?: THREE.Mesh[]
): number {
  const coneHalfAngle = 0.05;
  const numRays = 5;
  let closest = maxDist;

  for (let i = 0; i < numRays; i++) {
    const offset = (i / (numRays - 1) - 0.5) * coneHalfAngle;
    const angle = headingAngle + sensorAngle + offset;
    const dir = new THREE.Vector3(Math.cos(angle), Math.sin(angle), 0);
    const origin = new THREE.Vector3(x * 0.001, y * 0.001, 0.1);
    const raycaster = new THREE.Raycaster(origin, dir);
    raycaster.far = maxDist * 0.001;

    if (scene) {
      const hits = raycaster.intersectObjects(scene.children, true);
      for (const hit of hits) {
        if (hit.distance > 0.01) {
          const d = Math.round(hit.distance * MM_PER_M);
          if (d < closest) closest = d;
          break;
        }
      }
    }

    if (meshes && meshes.length > 0) {
      const hits = raycaster.intersectObjects(meshes);
      for (const hit of hits) {
        if (hit.distance > 0.01) {
          const d = Math.round(hit.distance * MM_PER_M);
          if (d < closest) closest = d;
          break;
        }
      }
    }
  }

  return closest;
}

function simulateUltraCone(
  x: number,
  y: number,
  headingAngle: number,
  sensorAngle: number,
  maxDist: number,
  scene?: THREE.Scene,
  meshes?: THREE.Mesh[]
): number {
  const coneHalfAngle = 0.2;
  const numRays = 7;
  let closest = maxDist;

  for (let i = 0; i < numRays; i++) {
    const offset = (i / (numRays - 1) - 0.5) * coneHalfAngle;
    const angle = headingAngle + sensorAngle + offset;
    const dir = new THREE.Vector3(Math.cos(angle), Math.sin(angle), 0);
    const origin = new THREE.Vector3(x * 0.001, y * 0.001, 0.08);
    const raycaster = new THREE.Raycaster(origin, dir);
    raycaster.far = maxDist * 0.001;

    let hitDistance = maxDist;

    if (scene) {
      const hits = raycaster.intersectObjects(scene.children, true);
      for (const hit of hits) {
        if (hit.distance > 0.01) {
          const faceNormal = hit.face?.normal.clone().transformDirection(hit.object.matrixWorld);
          if (faceNormal) {
            const dot = Math.abs(faceNormal.dot(dir));
            if (dot < 0.3 && Math.random() < 0.4) continue;
          }
          hitDistance = Math.round(hit.distance * MM_PER_M);
          break;
        }
      }
    }

    if (hitDistance === maxDist && meshes && meshes.length > 0) {
      const hits = raycaster.intersectObjects(meshes);
      for (const hit of hits) {
        if (hit.distance > 0.01) {
          const faceNormal = hit.face?.normal.clone().transformDirection(hit.object.matrixWorld);
          if (faceNormal) {
            const dot = Math.abs(faceNormal.dot(dir));
            if (dot < 0.3 && Math.random() < 0.4) continue;
          }
          hitDistance = Math.round(hit.distance * MM_PER_M);
          break;
        }
      }
    }

    if (hitDistance < closest) closest = hitDistance;
  }

  return closest;
}

function simulateLineSensor(
  worldX: number,
  worldY: number,
  headingRad: number,
  sensorX: number,
  sensorY: number,
  obstacles: THREE.Mesh[],
  scene?: THREE.Scene
): number {
  const sensorWorldX = worldX + sensorX * Math.cos(headingRad) - sensorY * Math.sin(headingRad);
  const sensorWorldY = worldY + sensorX * Math.sin(headingRad) + sensorY * Math.cos(headingRad);
  const gx = sensorWorldX * 0.001;
  const gy = sensorWorldY * 0.001;

  const raycaster = new THREE.Raycaster(
    new THREE.Vector3(gx, gy, 0.1),
    new THREE.Vector3(0, 0, -1)
  );

  let groundColor: THREE.Color | null = null;

  if (scene) {
    const hits = raycaster.intersectObjects(scene.children, true);
    for (const hit of hits) {
      if (hit.distance > 0.001 && hit.distance < 0.3) {
        if (hit.object.type === 'Mesh') {
          const mesh = hit.object as THREE.Mesh;
          const mat = mesh.material as THREE.MeshStandardMaterial;
          if (mat.color) {
            groundColor = mat.color;
            break;
          }
        }
      }
    }
  }

  if (obstacles.length > 0) {
    const hits = raycaster.intersectObjects(obstacles);
    for (const hit of hits) {
      if (hit.distance > 0.001 && hit.distance < 0.3) {
        const mat = (hit.object as THREE.Mesh).material as THREE.MeshStandardMaterial;
        if (mat && mat.color) {
          groundColor = mat.color;
          break;
        }
      }
    }
  }

  if (!groundColor) {
    const bounds = 5;
    const isOnFloor = Math.abs(gx) < bounds && Math.abs(gy) < bounds;
    if (isOnFloor) return 0;
    return 1023;
  }

  const r = groundColor.r;
  const g = groundColor.g;
  const b = groundColor.b;
  const luminance = 0.299 * r + 0.587 * g + 0.114 * b;

  if (luminance < 0.3) return 0;
  if (luminance > 0.7) return 1023;

  const noise = gaussianNoise(50);
  return Math.round(1023 * (1 - luminance) + noise);
}

export class MockSensorGenerator {
  private config: MockSensorConfig;
  private raycaster = new THREE.Raycaster();
  private obstacleMeshes: THREE.Mesh[] = [];
  private groundColorCache = new Map<string, number>();
  private imu: ImuState = createImuState();
  private driftAccum = 0;
  private lastTime = 0;

  constructor(config?: Partial<MockSensorConfig>) {
    this.config = { ...DEFAULT_CONFIG, ...config };
  }

  setObstacleMeshes(meshes: THREE.Mesh[]) {
    this.obstacleMeshes = meshes;
    this.groundColorCache.clear();
  }

  generate(robotState: KinematicState, scene?: THREE.Scene): MockSensorData {
    const headingRad = (robotState.heading * Math.PI) / 180;
    const rx = robotState.x;
    const ry = robotState.y;

    const now = performance.now();
    const dt = this.lastTime === 0 ? 1 / 30 : Math.min((now - this.lastTime) / 1000, 0.1);
    this.lastTime = now;

    this.driftAccum += gaussianNoise(noiseConfig.headingDriftDegPerSec * dt);
    this.driftAccum = Math.max(-2, Math.min(2, this.driftAccum));

    const sensors: Record<string, number> = {};

    const rawLasers: Record<string, number> = {};
    for (const pos of this.config.laserPositions) {
      const worldX = rx + pos.x * Math.cos(headingRad) - pos.y * Math.sin(headingRad);
      const worldY = ry + pos.x * Math.sin(headingRad) + pos.y * Math.cos(headingRad);
      const rayAngle = headingRad + pos.angle;
      const dist = simulateIrCone(
        worldX, worldY, rayAngle, 0, LASER_MAX_MM, scene, this.obstacleMeshes
      );
      rawLasers[pos.name] = dist;
      sensors[pos.name] = applyNoise(dist, noiseConfig.laserSigma, LASER_MAX_MM, true);
    }

    const rawUltras: Record<string, number> = {};
    for (const pos of this.config.ultraPositions) {
      const worldX = rx + pos.x * Math.cos(headingRad) - pos.y * Math.sin(headingRad);
      const worldY = ry + pos.x * Math.sin(headingRad) + pos.y * Math.cos(headingRad);
      const rayAngle = headingRad + pos.angle;
      const dist = simulateUltraCone(
        worldX, worldY, rayAngle, 0, ULTRASOUND_MAX_MM, scene, this.obstacleMeshes
      );
      rawUltras[pos.name] = dist;
      sensors[pos.name] = applyNoise(dist, noiseConfig.ultraSigma, ULTRASOUND_MAX_MM, true);
    }

    if (rawUltras['ultra_front_left'] < noiseConfig.ultraCrosstalkThreshold &&
        rawUltras['ultra_front_right'] < noiseConfig.ultraCrosstalkThreshold) {
      const crosstalk = noiseConfig.ultraCrosstalkInfluence *
        Math.abs(sensors['ultra_front_left'] - sensors['ultra_front_right']);
      sensors['ultra_front_left'] = Math.round(
        sensors['ultra_front_left'] + crosstalk * (Math.random() > 0.5 ? 1 : -1)
      );
      sensors['ultra_front_right'] = Math.round(
        sensors['ultra_front_right'] + crosstalk * (Math.random() > 0.5 ? 1 : -1)
      );
    }

    const lineKeys = ['line_left', 'line_center', 'line_right'] as const;
    for (let i = 0; i < this.config.linePositions.length; i++) {
      const pos = this.config.linePositions[i];
      const raw = simulateLineSensor(
        rx, ry, headingRad, pos.x, pos.y, this.obstacleMeshes, scene
      );
      if (Math.random() < noiseConfig.lineFlipProb) {
        sensors[lineKeys[i]] = raw < 512 ? 1023 : 0;
      } else {
        sensors[lineKeys[i]] = raw;
      }
    }

    const imuReading = simulateImu(
      this.imu,
      robotState.heading,
      robotState.vx,
      robotState.vy,
      dt,
      this.driftAccum
    );

    return {
      laser_left_front: sensors['laser_left_front'] ?? LASER_MAX_MM,
      laser_left_back: sensors['laser_left_back'] ?? LASER_MAX_MM,
      laser_right_front: sensors['laser_right_front'] ?? LASER_MAX_MM,
      laser_right_back: sensors['laser_right_back'] ?? LASER_MAX_MM,
      laser_back_left: sensors['laser_back_left'] ?? LASER_MAX_MM,
      laser_back_right: sensors['laser_back_right'] ?? LASER_MAX_MM,
      ultra_front_left: sensors['ultra_front_left'] ?? ULTRASOUND_MAX_MM,
      ultra_front_right: sensors['ultra_front_right'] ?? ULTRASOUND_MAX_MM,
      line_left: sensors['line_left'] ?? 1023,
      line_center: sensors['line_center'] ?? 1023,
      line_right: sensors['line_right'] ?? 1023,
      imu_heading: imuReading.heading,
      imu_pitch: imuReading.pitch,
      imu_roll: imuReading.roll,
    };
  }
}
