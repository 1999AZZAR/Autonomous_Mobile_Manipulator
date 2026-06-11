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

export class MockSensorGenerator {
  private config: MockSensorConfig;
  private raycaster = new THREE.Raycaster();
  private groundPlane = new THREE.Plane(new THREE.Vector3(0, 0, 1), 0);
  private obstacleMeshes: THREE.Mesh[] = [];
  private lastHeading = 0;
  private driftAccum = 0;

  constructor(config?: Partial<MockSensorConfig>) {
    this.config = { ...DEFAULT_CONFIG, ...config };
  }

  setObstacleMeshes(meshes: THREE.Mesh[]) {
    this.obstacleMeshes = meshes;
  }

  generate(robotState: KinematicState, scene?: THREE.Scene): MockSensorData {
    const headingRad = (robotState.heading * Math.PI) / 180;
    const rx = robotState.x;
    const ry = robotState.y;

    const dt = 1 / 30;
    this.driftAccum += gaussianNoise(noiseConfig.headingDriftDegPerSec * dt);
    this.driftAccum = Math.max(-2, Math.min(2, this.driftAccum));

    const sensors: Record<string, number> = {};

    const rawLasers: Record<string, number> = {};
    this.config.laserPositions.forEach((pos) => {
      const worldX = rx + pos.x * Math.cos(headingRad) - pos.y * Math.sin(headingRad);
      const worldY = ry + pos.x * Math.sin(headingRad) + pos.y * Math.cos(headingRad);
      const rayAngle = headingRad + pos.angle;
      const dist = this.castRay(worldX, worldY, rayAngle, LASER_MAX_MM, scene);
      rawLasers[pos.name] = dist;
      sensors[pos.name] = applyNoise(dist, noiseConfig.laserSigma, LASER_MAX_MM, true);
    });

    const rawUltras: Record<string, number> = {};
    this.config.ultraPositions.forEach((pos) => {
      const worldX = rx + pos.x * Math.cos(headingRad) - pos.y * Math.sin(headingRad);
      const worldY = ry + pos.x * Math.sin(headingRad) + pos.y * Math.cos(headingRad);
      const rayAngle = headingRad + pos.angle;
      const dist = this.castRay(worldX, worldY, rayAngle, ULTRASOUND_MAX_MM, scene);
      rawUltras[pos.name] = dist;
      sensors[pos.name] = applyNoise(dist, noiseConfig.ultraSigma, ULTRASOUND_MAX_MM, true);
    });

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

    const lineThreshold = 150;
    this.config.linePositions.forEach((pos, i) => {
      const worldY = ry + pos.y * Math.cos(headingRad) + pos.x * Math.sin(headingRad);
      const keys = ['line_left', 'line_center', 'line_right'] as const;
      let raw = Math.abs(worldY) < lineThreshold ? 0 : 1023;
      if (Math.abs(worldY - lineThreshold) < 20 && Math.random() < noiseConfig.lineFlipProb) {
        raw = raw === 0 ? 1023 : 0;
      }
      sensors[keys[i]] = raw;
    });

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
      imu_heading: robotState.heading + this.driftAccum,
      imu_pitch: 0,
      imu_roll: 0,
    };
  }

  private castRay(x: number, y: number, angle: number, maxDist: number, scene?: THREE.Scene): number {
    const dir = new THREE.Vector3(Math.cos(angle), Math.sin(angle), 0);
    const origin = new THREE.Vector3(x * 0.001, y * 0.001, 0.1);

    this.raycaster.set(origin, dir);
    this.raycaster.far = maxDist * 0.001;

    if (scene) {
      const intersects = this.raycaster.intersectObjects(scene.children, true);
      for (const hit of intersects) {
        if (hit.distance > 0.01) return Math.round(hit.distance * MM_PER_M);
      }
    }

    if (this.obstacleMeshes.length > 0) {
      const intersects = this.raycaster.intersectObjects(this.obstacleMeshes);
      for (const hit of intersects) {
        if (hit.distance > 0.01) return Math.round(hit.distance * MM_PER_M);
      }
    }

    return maxDist;
  }
}
