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
    description: '10x10m room, no obstacles',
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
    name: 'Warehouse',
    description: '3 shelf aisles with narrow passages',
    scenario: {
      name: 'Warehouse',
      description: 'Simulated warehouse with shelf aisles',
      floorSize: 10000,
      robotStart: { x: -4000, y: 0, heading: 0, vx: 0, vy: 0, omega: 0 },
      objects: [
        // Shelf rows
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
    description: 'Table + objects for manipulation',
    scenario: {
      name: 'Pick and Place',
      description: 'Objects on a table for pick-and-place tasks',
      floorSize: 10000,
      robotStart: { x: 0, y: -2000, heading: 0, vx: 0, vy: 0, omega: 0 },
      objects: [
        // Table
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
        // Objects on table
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
    description: 'Wall maze with narrow corridors',
    scenario: {
      name: 'Maze',
      description: 'Simple maze with walls',
      floorSize: 10000,
      robotStart: { x: -4000, y: -4000, heading: 0, vx: 0, vy: 0, omega: 0 },
      objects: [
        // Outer walls (boundaries handled separately)
        // Inner walls
        { id: 'wall_1', type: 'obstacle', x: -2000, y: -1000, width: 4000, height: 100, depth: 300, obstacleType: 'box', color: 0x555555 },
        { id: 'wall_2', type: 'obstacle', x: 2000, y: 1000, width: 4000, height: 100, depth: 300, obstacleType: 'box', color: 0x555555 },
        { id: 'wall_3', type: 'obstacle', x: 0, y: 3000, width: 100, height: 4000, depth: 300, obstacleType: 'box', color: 0x555555 },
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
