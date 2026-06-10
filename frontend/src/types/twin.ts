// Digital twin specific types

import type { RobotPosition, SensorReadings } from '../types';
import type * as THREE from 'three';

export interface TwinState {
  position: RobotPosition;
  heading: number;
  sensors: SensorReadings;
  gripperOpen: boolean;
  tiltAngle: number;
  lifterHeight: number;
  connected: boolean;
  mode: 'replay' | 'ifttt' | 'ai' | 'idle';
}

export interface Obstacle {
  id: string;
  x: number;
  y: number;
  width: number;
  height: number;
  depth: number;
  type: 'box' | 'cylinder';
  color?: number;
}

export interface Waypoint3D {
  x: number;
  y: number;
  z: number;
  heading: number;
  order: number;
  actions?: Record<string, unknown>;
}

export interface SensorArc {
  sensor: string;
  distance: number;
  maxRange: number;
  angle: number;
  position: { x: number; y: number; z: number };
  active: boolean;
}

export interface RobotModelParts {
  body: { rotation: { x: number } };
  wheels: Array<{ rotation: { y: number } }>;
  gripper: { left: { position: { x: number } }; right: { position: { x: number } } };
  gripperAssembly: THREE.Group;
  tiltServo: { rotation: { x: number } };
  laserSensors: Array<{ material: any }>;
  ultraSensors: Array<{ material: any }>;
  lineSensors: Array<{ material: any }>;
  headingArrow: { material: any };
}
