// Scenario runner — save/load/preview scenarios, competition presets

import * as THREE from 'three';
import type { Obstacle } from '../types/twin';
import type { KinematicState } from './physics';

export interface ScenarioObject {
  id: string;
  type: 'obstacle' | 'target' | 'waypoint';
  x: number;
  y: number;
  width: number;
  height: number;
  depth: number;
  obstacleType?: 'box' | 'cylinder';
  color?: number;
}

export interface Scenario {
  name: string;
  description: string;
  floorSize: number;          // mm
  robotStart: KinematicState;
  objects: ScenarioObject[];
  createdAt: string;
}

export interface ScenarioPreset {
  name: string;
  description: string;
  scenario: Scenario;
}

// Competition layout presets
export const PRESETS: ScenarioPreset[] = [
  {
    name: 'Empty Room',
    description: '10x10m open area, no obstacles — free driving',
    scenario: {
      name: 'Empty Room',
      description: 'Empty 10x10m room with robot at center',
      floorSize: 10000,
      robotStart: { x: 0, y: 0, heading: 0, vx: 0, vy: 0, omega: 0 },
      objects: [],
      createdAt: new Date().toISOString(),
    },
  },
  {
    name: 'Waypoint Circuit',
    description: 'Obstacle course — navigate around barriers in sequence',
    scenario: {
      name: 'Waypoint Circuit',
      description: 'Practice waypoint navigation around barriers',
      floorSize: 10000,
      robotStart: { x: -3500, y: -3500, heading: 0, vx: 0, vy: 0, omega: 0 },
      objects: [
        { id: 'barrier_1', type: 'obstacle', x: 0, y: 0, width: 300, height: 200, depth: 3000, obstacleType: 'box', color: 0xcc4444 },
        { id: 'barrier_2', type: 'obstacle', x: -2500, y: 2000, width: 3000, height: 200, depth: 300, obstacleType: 'box', color: 0x44cc44 },
        { id: 'barrier_3', type: 'obstacle', x: 2500, y: -2000, width: 3000, height: 200, depth: 300, obstacleType: 'box', color: 0x4444cc },
      ],
      createdAt: new Date().toISOString(),
    },
  },
  {
    name: 'Warehouse',
    description: '3 shelf aisles with narrow passages — practice precision driving',
    scenario: {
      name: 'Warehouse',
      description: 'Simulated warehouse with shelf aisles',
      floorSize: 10000,
      robotStart: { x: -4000, y: 0, heading: 0, vx: 0, vy: 0, omega: 0 },
      objects: [
        ...Array.from({ length: 3 }, (_, i) => ({
          id: `shelf_${i}_left`,
          type: 'obstacle' as const,
          x: -2000,
          y: -2000 + i * 2000,
          width: 1500,
          height: 200,
          depth: 300,
          obstacleType: 'box' as const,
          color: 0x8b4513,
        })),
        ...Array.from({ length: 3 }, (_, i) => ({
          id: `shelf_${i}_right`,
          type: 'obstacle' as const,
          x: 2000,
          y: -2000 + i * 2000,
          width: 1500,
          height: 200,
          depth: 300,
          obstacleType: 'box' as const,
          color: 0x8b4513,
        })),
      ],
      createdAt: new Date().toISOString(),
    },
  },
  {
    name: 'Pick and Place',
    description: 'Table + manipulation targets for arm practice',
    scenario: {
      name: 'Pick and Place',
      description: 'Objects on a table for pick-and-place tasks',
      floorSize: 10000,
      robotStart: { x: 0, y: -2000, heading: 0, vx: 0, vy: 0, omega: 0 },
      objects: [
        {
          id: 'table',
          type: 'obstacle',
          x: 0,
          y: 1500,
          width: 2000,
          height: 500,
          depth: 50,
          obstacleType: 'box',
          color: 0x654321,
        },
        {
          id: 'cup_1',
          type: 'target',
          x: -400,
          y: 1500,
          width: 80,
          height: 80,
          depth: 100,
          obstacleType: 'cylinder',
          color: 0xff4444,
        },
        {
          id: 'cup_2',
          type: 'target',
          x: 200,
          y: 1300,
          width: 80,
          height: 80,
          depth: 100,
          obstacleType: 'cylinder',
          color: 0x4444ff,
        },
      ],
      createdAt: new Date().toISOString(),
    },
  },
  {
    name: 'Maze',
    description: 'Wall maze with narrow corridors — test pathfinding',
    scenario: {
      name: 'Maze',
      description: 'Simple maze with walls',
      floorSize: 10000,
      robotStart: { x: -4000, y: -4000, heading: 0, vx: 0, vy: 0, omega: 0 },
      objects: [
        { id: 'wall_1', type: 'obstacle', x: -2000, y: -1000, width: 4000, height: 100, depth: 300, obstacleType: 'box', color: 0x555555 },
        { id: 'wall_2', type: 'obstacle', x: 2000, y: 1000, width: 4000, height: 100, depth: 300, obstacleType: 'box', color: 0x555555 },
        { id: 'wall_3', type: 'obstacle', x: 0, y: 3000, width: 100, height: 4000, depth: 300, obstacleType: 'box', color: 0x555555 },
      ],
      createdAt: new Date().toISOString(),
    },
  },
  {
    name: 'Obstacle Course',
    description: 'Random obstacles — test collision avoidance',
    scenario: {
      name: 'Obstacle Course',
      description: 'Scattered obstacles for collision avoidance testing',
      floorSize: 10000,
      robotStart: { x: -4000, y: -4000, heading: 0, vx: 0, vy: 0, omega: 0 },
      objects: [
        { id: 'obs_1', type: 'obstacle', x: -2000, y: -2000, width: 400, height: 200, depth: 400, obstacleType: 'box', color: 0xcc6633 },
        { id: 'obs_2', type: 'obstacle', x: 1500, y: -1000, width: 500, height: 200, depth: 500, obstacleType: 'box', color: 0x33cc66 },
        { id: 'obs_3', type: 'obstacle', x: -1000, y: 2000, width: 300, height: 200, depth: 300, obstacleType: 'box', color: 0x6633cc },
        { id: 'obs_4', type: 'obstacle', x: 3000, y: 1500, width: 600, height: 200, depth: 600, obstacleType: 'box', color: 0xcc3366 },
        { id: 'obs_5', type: 'obstacle', x: 0, y: -3000, width: 2000, height: 100, depth: 200, obstacleType: 'box', color: 0x66cc33 },
      ],
      createdAt: new Date().toISOString(),
    },
  },
];

// LocalStorage key
const STORAGE_KEY = 'amm-twin-scenarios';

export function saveScenario(scenario: Scenario): void {
  const scenarios = loadAllScenarios();
  const existing = scenarios.findIndex((s) => s.name === scenario.name);
  if (existing >= 0) {
    scenarios[existing] = scenario;
  } else {
    scenarios.push(scenario);
  }
  localStorage.setItem(STORAGE_KEY, JSON.stringify(scenarios));
}

export function loadAllScenarios(): Scenario[] {
  try {
    const data = localStorage.getItem(STORAGE_KEY);
    return data ? JSON.parse(data) : [];
  } catch {
    return [];
  }
}

export function loadScenario(name: string): Scenario | null {
  const scenarios = loadAllScenarios();
  return scenarios.find((s) => s.name === name) || null;
}

export function deleteScenario(name: string): void {
  const scenarios = loadAllScenarios().filter((s) => s.name !== name);
  localStorage.setItem(STORAGE_KEY, JSON.stringify(scenarios));
}

export function scenarioToObstacles(scenario: Scenario): Obstacle[] {
  return scenario.objects
    .filter((obj) => obj.type === 'obstacle')
    .map((obj) => ({
      id: obj.id,
      x: obj.x,
      y: obj.y,
      width: obj.width,
      height: obj.height,
      depth: obj.depth,
      type: obj.obstacleType || 'box',
      color: obj.color,
    }));
}

export function scenarioToObstacleMeshes(
  scenario: Scenario,
  createMesh: (obs: Obstacle) => THREE.Mesh
): THREE.Mesh[] {
  return scenarioToObstacles(scenario).map(createMesh);
}

// --- Example waypoint paths for each scenario (coordinates in meters) ---

export interface ExamplePath {
  scenarioName: string;
  name: string;
  waypoints: Array<{ x: number; y: number; heading: number }>;
}

export const EXAMPLE_PATHS: ExamplePath[] = [
  {
    scenarioName: 'Waypoint Circuit',
    name: 'Circuit Lap',
    waypoints: [
      { x: -3.5, y: -3.5, heading: 0 },
      { x: -2.0, y: -2.5, heading: 0 },
      { x: 0.0, y: -2.5, heading: 90 },
      { x: 2.0, y: -2.5, heading: 0 },
      { x: 3.5, y: -2.5, heading: 90 },
      { x: 3.5, y: 0.0, heading: 90 },
      { x: 3.5, y: 2.5, heading: 90 },
      { x: 2.0, y: 2.5, heading: 180 },
      { x: 0.0, y: 2.5, heading: 180 },
      { x: -2.0, y: 2.5, heading: 180 },
      { x: -3.5, y: 2.5, heading: -90 },
      { x: -3.5, y: 0.0, heading: -90 },
      { x: -3.5, y: -3.5, heading: -90 },
    ],
  },
  {
    scenarioName: 'Warehouse',
    name: 'Aisle Run',
    waypoints: [
      { x: -4.0, y: 0.0, heading: 0 },
      { x: -1.0, y: 0.0, heading: 0 },
      { x: -1.0, y: -2.5, heading: -90 },
      { x: 1.0, y: -2.5, heading: 0 },
      { x: 1.0, y: 0.0, heading: 90 },
      { x: 1.0, y: 2.5, heading: 90 },
      { x: -1.0, y: 2.5, heading: 180 },
      { x: -1.0, y: 0.0, heading: 180 },
      { x: -4.0, y: 0.0, heading: 180 },
    ],
  },
  {
    scenarioName: 'Empty Room',
    name: 'Figure Eight',
    waypoints: [
      { x: 0.0, y: 0.0, heading: 0 },
      { x: 3.0, y: 0.0, heading: 0 },
      { x: 3.0, y: 3.0, heading: 90 },
      { x: 0.0, y: 3.0, heading: 180 },
      { x: -3.0, y: 3.0, heading: 180 },
      { x: -3.0, y: 0.0, heading: -90 },
      { x: -3.0, y: -3.0, heading: -90 },
      { x: 0.0, y: -3.0, heading: 0 },
      { x: 0.0, y: 0.0, heading: 0 },
    ],
  },
  {
    scenarioName: 'Obstacle Course',
    name: 'Slalom',
    waypoints: [
      { x: -4.0, y: -4.0, heading: 0 },
      { x: -2.5, y: -3.0, heading: 45 },
      { x: -1.0, y: -2.5, heading: 0 },
      { x: 1.5, y: -2.5, heading: 0 },
      { x: 3.5, y: -1.0, heading: 90 },
      { x: 3.5, y: 1.0, heading: 90 },
      { x: 2.0, y: 2.5, heading: 135 },
      { x: -1.5, y: 3.0, heading: 180 },
      { x: -3.5, y: 1.5, heading: -90 },
      { x: -3.5, y: -1.0, heading: -90 },
      { x: -3.5, y: -3.5, heading: -90 },
    ],
  },
  {
    scenarioName: 'Maze',
    name: 'Maze Run',
    waypoints: [
      { x: -4.0, y: -4.0, heading: 0 },
      { x: 2.0, y: -2.0, heading: 0 },
      { x: 2.0, y: 0.0, heading: 90 },
      { x: 2.0, y: 2.0, heading: 90 },
      { x: -0.5, y: 2.0, heading: 180 },
      { x: -0.5, y: 4.0, heading: 90 },
      { x: 3.0, y: 4.0, heading: 0 },
    ],
  },
];
