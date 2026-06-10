// Shared digital twin state

import type { TwinState, Obstacle, Waypoint3D } from '../types/twin';
import type { SensorData, RobotPosition } from '../types';

const defaultState: TwinState = {
  position: { x: 0, y: 0, z: 0, roll: 0, pitch: 0, yaw: 0 },
  heading: 0,
  sensors: {
    laser_left_front: 1500,
    laser_left_back: 1500,
    laser_right_front: 1500,
    laser_right_back: 1500,
    laser_back_left: 1500,
    laser_back_right: 1500,
    ultra_front_left: 4000,
    ultra_front_right: 4000,
  },
  gripperOpen: true,
  tiltAngle: 90,
  armJoints: [0, 0, 0, 0, 0, 0],
  connected: false,
  mode: 'idle',
};

let state: TwinState = { ...defaultState };
let obstacles: Obstacle[] = [];
let waypoints: Waypoint3D[] = [];
let listeners: Array<(state: TwinState) => void> = [];

export function getTwinState(): Readonly<TwinState> {
  return state;
}

export function updateTwinState(partial: Partial<TwinState>) {
  state = { ...state, ...partial };
  listeners.forEach((fn) => fn(state));
}

export function updateFromSensorData(data: SensorData) {
  state = {
    ...state,
    sensors: {
      laser_left_front: data.laser_left_front,
      laser_left_back: data.laser_left_back,
      laser_right_front: data.laser_right_front,
      laser_right_back: data.laser_right_back,
      laser_back_left: data.laser_back_left,
      laser_back_right: data.laser_back_right,
      ultra_front_left: data.ultra_front_left,
      ultra_front_right: data.ultra_front_right,
    },
    heading: data.imu_heading,
    connected: data.mega_connected,
  };
  listeners.forEach((fn) => fn(state));
}

export function updateRobotPosition(pos: RobotPosition) {
  state = {
    ...state,
    position: pos,
    heading: pos.yaw,
  };
  listeners.forEach((fn) => fn(state));
}

export function onTwinStateChange(fn: (state: TwinState) => void): () => void {
  listeners.push(fn);
  return () => {
    listeners = listeners.filter((h) => h !== fn);
  };
}

export function getObstacles(): readonly Obstacle[] {
  return obstacles;
}

export function setObstacles(newObstacles: Obstacle[]) {
  obstacles = newObstacles;
}

export function getWaypoints(): readonly Waypoint3D[] {
  return waypoints;
}

export function setWaypoints(newWaypoints: Waypoint3D[]) {
  waypoints = newWaypoints;
}
