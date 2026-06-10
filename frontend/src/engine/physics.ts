// Simple kinematic simulation — position, velocity, collision detection

import type { Obstacle, TwinState } from '../types/twin';

const MM_PER_M = 1000;
const DT_DEFAULT = 1 / 60; // 60 Hz

export interface KinematicState {
  x: number;       // mm
  y: number;       // mm
  heading: number;  // degrees
  vx: number;       // mm/s
  vy: number;       // mm/s
  omega: number;    // deg/s
}

export interface SimulationConfig {
  maxSpeed: number;        // mm/s
  maxAngularSpeed: number; // deg/s
  acceleration: number;    // mm/s^2
  deceleration: number;    // mm/s^2
  turnSpeed: number;       // deg/s
}

const DEFAULT_CONFIG: SimulationConfig = {
  maxSpeed: 500,
  maxAngularSpeed: 120,
  acceleration: 400,
  deceleration: 600,
  turnSpeed: 90,
};

export class PhysicsEngine {
  private state: KinematicState = { x: 0, y: 0, heading: 0, vx: 0, vy: 0, omega: 0 };
  private config: SimulationConfig;
  private obstacles: Obstacle[] = [];
  private targetCommand: { vx: number; vy: number; omega: number } | null = null;
  private robotRadius = 220; // mm (hexagonal body ~0.22m)

  constructor(config?: Partial<SimulationConfig>) {
    this.config = { ...DEFAULT_CONFIG, ...config };
  }

  getState(): Readonly<KinematicState> {
    return this.state;
  }

  setState(s: KinematicState) {
    this.state = { ...s };
  }

  setObstacles(obstacles: Obstacle[]) {
    this.obstacles = obstacles;
  }

  command(vx: number, vy: number, omega: number) {
    this.targetCommand = { vx, vy, omega };
  }

  stop() {
    this.targetCommand = { vx: 0, vy: 0, omega: 0 };
  }

  step(dt: number = DT_DEFAULT): KinematicState {
    if (this.targetCommand) {
      // Smooth acceleration toward target velocity
      const accel = this.config.acceleration * dt;
      const decel = this.config.deceleration * dt;

      // X velocity
      const dvx = this.targetCommand.vx - this.state.vx;
      if (Math.abs(dvx) > 0) {
        const limit = dvx > 0 ? accel : decel;
        this.state.vx += Math.sign(dvx) * Math.min(Math.abs(dvx), limit);
      }

      // Y velocity
      const dvy = this.targetCommand.vy - this.state.vy;
      if (Math.abs(dvy) > 0) {
        const limit = dvy > 0 ? accel : decel;
        this.state.vy += Math.sign(dvy) * Math.min(Math.abs(dvy), limit);
      }

      // Angular velocity
      const domega = this.targetCommand.omega - this.state.omega;
      if (Math.abs(domega) > 0) {
        const limit = this.config.turnSpeed * dt;
        this.state.omega += Math.sign(domega) * Math.min(Math.abs(domega), limit);
      }
    } else {
      // Decelerate to stop
      const decel = this.config.deceleration * dt;
      this.state.vx = decelerate(this.state.vx, decel);
      this.state.vy = decelerate(this.state.vy, decel);
      this.state.omega = decelerate(this.state.omega, this.config.turnSpeed * dt);
    }

    // Clamp speeds
    this.state.vx = clamp(this.state.vx, -this.config.maxSpeed, this.config.maxSpeed);
    this.state.vy = clamp(this.state.vy, -this.config.maxSpeed, this.config.maxSpeed);
    this.state.omega = clamp(this.state.omega, -this.config.maxAngularSpeed, this.config.maxAngularSpeed);

    // Integrate position
    const newX = this.state.x + this.state.vx * dt;
    const newY = this.state.y + this.state.vy * dt;
    const newHeading = this.state.heading + this.state.omega * dt;

    // Collision detection
    if (!this.checkCollision(newX, newY)) {
      this.state.x = newX;
      this.state.y = newY;
    } else {
      // Stop on collision
      this.state.vx = 0;
      this.state.vy = 0;
    }

    // Normalize heading
    this.state.heading = ((newHeading % 360) + 360) % 360;

    return { ...this.state };
  }

  private checkCollision(x: number, y: number): boolean {
    for (const obs of this.obstacles) {
      const ox = obs.x;
      const oy = obs.y;
      const hw = obs.width / 2 + this.robotRadius;
      const hh = (obs.type === 'cylinder' ? obs.width : obs.height) / 2 + this.robotRadius;

      if (Math.abs(x - ox) < hw && Math.abs(y - oy) < hh) {
        return true;
      }
    }
    return false;
  }

  // Odometry: simulate wheel encoder deltas
  getOdometryDelta(dt: number): { dl: number; dr: number; db: number } {
    const headingRad = (this.state.heading * Math.PI) / 180;
    // Transform global velocity to robot frame
    const vrx = this.state.vx * Math.cos(headingRad) + this.state.vy * Math.sin(headingRad);
    const vry = -this.state.vx * Math.sin(headingRad) + this.state.vy * Math.cos(headingRad);

    // 3-wheel omni: each wheel sees different component
    const wheelAngles = [0, (2 * Math.PI) / 3, (4 * Math.PI) / 3];
    const deltas = wheelAngles.map((a) => {
      const vWheel = vrx * Math.cos(a) + vry * Math.sin(a);
      return vWheel * dt; // mm per tick
    });

    return { dl: deltas[0], dr: deltas[1], db: deltas[2] };
  }
}

function decelerate(v: number, decel: number): number {
  if (Math.abs(v) < decel) return 0;
  return v - Math.sign(v) * decel;
}

function clamp(v: number, min: number, max: number): number {
  return Math.max(min, Math.min(max, v));
}
