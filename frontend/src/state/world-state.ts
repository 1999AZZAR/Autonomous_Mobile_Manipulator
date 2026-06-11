import type { RobotPosition, SensorReadings } from '../types';

export interface SensorRay {
  origin: { x: number; y: number };
  endpoint: { x: number; y: number };
  distance: number;
  label: string;
}

export interface OccupancyCell {
  x: number;
  y: number;
  occupied: boolean;
}

export interface WorldStateData {
  robotPosition: { x: number; y: number; heading: number };
  robotPosition3D: RobotPosition;
  occupancyGrid: Float32Array;
  gridWidth: number;
  gridHeight: number;
  gridOriginX: number;
  gridOriginY: number;
  obstacles: Array<{ x: number; y: number; width: number; height: number; depth: number }>;
  waypoints: Array<{ x: number; y: number }>;
  sensorRays: SensorRay[];
  sensors: Partial<SensorReadings>;
  timestamp: number;
  source?: string;
}

type WorldListener = (data: WorldStateData) => void;
type WorldSource = 'sim' | 'real' | 'map-draw' | 'twin-interact';

class WorldStateManager {
  private data: WorldStateData = {
    robotPosition: { x: 0, y: 0, heading: 0 },
    robotPosition3D: { x: 0, y: 0, z: 0, roll: 0, pitch: 0, yaw: 0 },
    occupancyGrid: new Float32Array(3000),
    gridWidth: 60,
    gridHeight: 50,
    gridOriginX: 0,
    gridOriginY: 0,
    obstacles: [],
    waypoints: [],
    sensorRays: [],
    sensors: {},
    timestamp: 0,
    source: '',
  };

  private listeners = new Map<string, WorldListener>();

  get(): WorldStateData {
    return this.data;
  }

  update(source: WorldSource, partial: Partial<WorldStateData>) {
    Object.assign(this.data, partial);
    this.data.timestamp = Date.now();
    this.data.source = source;
    this.notifyAll();
  }

  setOccupancyCell(gx: number, gy: number, occupied: boolean) {
    const idx = gy * this.data.gridWidth + gx;
    if (idx >= 0 && idx < this.data.occupancyGrid.length) {
      this.data.occupancyGrid[idx] = occupied ? 1 : 0;
    }
  }

  isOccupied(gx: number, gy: number): boolean {
    const idx = gy * this.data.gridWidth + gx;
    if (idx >= 0 && idx < this.data.occupancyGrid.length) {
      return this.data.occupancyGrid[idx] > 0.5;
    }
    return false;
  }

  addObstacle(obs: { x: number; y: number; width: number; height: number; depth: number }) {
    this.data.obstacles.push(obs);
    this.notifyAll();
  }

  removeAllObstacles() {
    this.data.obstacles = [];
    this.data.occupancyGrid.fill(0);
    this.notifyAll();
  }

  subscribe(id: string, listener: WorldListener) {
    this.listeners.set(id, listener);
    listener({ ...this.data });
  }

  unsubscribe(id: string) {
    this.listeners.delete(id);
  }

  private notifyAll() {
    const snapshot = { ...this.data };
    this.listeners.forEach((cb) => cb(snapshot));
  }
}

export const WorldState = new WorldStateManager();
