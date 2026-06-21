// Kinematic simulation for a 3-wheel holonomic omni robot.
//
// Robot frame: +Y = forward, +X = right, +Z = up (Z-up, ROS2 convention).
// Wheels:  FR at 30° (Motor2), FL at 150° (Motor3), Back at 270° (Motor4).
//
// For a 3-wheel omni robot the velocity of each wheel is:
//   v_wheel_i = -sin(θ_i) * vx_robot + cos(θ_i) * vy_robot + R * omega
// where θ_i is the wheel angle, R is the robot radius, vx/vy are robot-local,
// and omega is the rotation rate (rad/s).
//
// Movement commands (vx, vy, omega) are given in the ROBOT frame.
// The physics engine transforms them to world frame using the heading.

import type { Obstacle } from '../types/twin';

const DEG2RAD = Math.PI / 180;

export interface KinematicState {
  x: number;       // mm  (world)
  y: number;       // mm  (world)
  heading: number; // degrees (world, 0=facing +Y)
  vx: number;      // mm/s (robot local, +X = right)
  vy: number;      // mm/s (robot local, +Y = forward)
  omega: number;   // deg/s (positive = CCW / left turn)
}

export interface SimulationConfig {
  maxSpeed: number;        // mm/s
  maxAngularSpeed: number; // deg/s
  acceleration: number;    // mm/s² (how fast velocity ramps up)
  deceleration: number;    // mm/s² (how fast it brakes)
}

const DEFAULT_CONFIG: SimulationConfig = {
  maxSpeed:        500,
  maxAngularSpeed: 150,
  acceleration:    600,
  deceleration:    900,
};

// 3 wheel angles in the robot frame (degrees from +X axis, CCW)
const WHEEL_ANGLES_DEG = [30, 150, 270]; // FR, FL, Back

export class PhysicsEngine {
  private state: KinematicState = { x: 0, y: 0, heading: 0, vx: 0, vy: 0, omega: 0 };
  private config: SimulationConfig;
  private obstacles: Obstacle[] = [];
  private targetVx  = 0;  // robot local
  private targetVy  = 0;
  private targetOmega = 0;
  private robotRadius = 230; // mm

  constructor(config?: Partial<SimulationConfig>) {
    this.config = { ...DEFAULT_CONFIG, ...config };
  }

  getState(): Readonly<KinematicState> { return this.state; }
  setState(s: KinematicState)          { this.state = { ...s }; }
  setObstacles(obs: Obstacle[]) {
    this.obstacles = obs;
    if (obs.length > 0) {
      console.log(`[Physics] setObstacles(${obs.length})`, obs.map(o => `${o.id}@(${o.x},${o.y}) w=${o.width} d=${o.depth} h=${o.height}`));
    } else {
      console.warn(`[Physics] setObstacles(0) — no obstacles!`);
    }
  }
  /** True if the robot would collide at the given position */
  wouldCollide(x: number, y: number): boolean {
    return this.checkCollision(x, y);
  }

  /** Command in robot-local frame: +vx=strafe-right, +vy=forward, +omega=CCW */
  command(vx: number, vy: number, omega: number) {
    this.targetVx    = vx;
    this.targetVy    = vy;
    this.targetOmega = omega;
    console.log(`[Physics] command: vx=${vx.toFixed(1)} vy=${vy.toFixed(1)} ω=${omega.toFixed(1)} | pos=(${this.state.x.toFixed(0)},${this.state.y.toFixed(0)}) h=${this.state.heading.toFixed(0)}`);
  }

  stop() {
    this.targetVx = this.targetVy = this.targetOmega = 0;
  }

  step(dt: number): KinematicState {
    const { acceleration: accel, deceleration: decel, maxSpeed, maxAngularSpeed } = this.config;

    // Ramp robot-local velocities toward target
    this.state.vx    = ramp(this.state.vx,    this.targetVx,    accel * dt, decel * dt);
    this.state.vy    = ramp(this.state.vy,    this.targetVy,    accel * dt, decel * dt);
    this.state.omega = ramp(this.state.omega, this.targetOmega, maxAngularSpeed * dt * 3, maxAngularSpeed * dt * 3);

    // Clamp
    this.state.vx    = clamp(this.state.vx,    -maxSpeed,        maxSpeed);
    this.state.vy    = clamp(this.state.vy,    -maxSpeed,        maxSpeed);
    this.state.omega = clamp(this.state.omega, -maxAngularSpeed, maxAngularSpeed);

    // Rotate robot-local velocity into world frame
    const h = this.state.heading * DEG2RAD;
    const cosH = Math.cos(h);
    const sinH = Math.sin(h);
    // robot +Y = forward, which maps to world (+cosH·Y + sinH·X ... wait:
    // heading 0 = facing world +Y. +vy_robot → world +Y, +vx_robot → world +X at heading=0.
    // After rotation by heading:
    //   world_x = vx*cos(h) - vy*sin(h)   ... but heading is CCW from +Y so:
    //   world_x = vx*cos(h) + vy*sin(h)
    //   world_y = -vx*sin(h) + vy*cos(h)
    // Let's verify: heading=0, vy=1 → world_x=0, world_y=1 ✓ (forward = +Y)
    //               heading=90, vy=1 → world_x=1, world_y=0 ✓ (forward = +X after 90°CCW turn)
    const worldVx = this.state.vx * cosH + this.state.vy * sinH;
    const worldVy = -this.state.vx * sinH + this.state.vy * cosH;

    const newX       = this.state.x       + worldVx * dt;
    const newY       = this.state.y       + worldVy * dt;
    const newHeading = (this.state.heading + this.state.omega * dt + 360) % 360;

    // Simple AABB collision
    const blocked = this.checkCollision(newX, newY);
    if (!blocked) {
      this.state.x = newX;
      this.state.y = newY;
    } else {
      console.warn(`[Physics] BLOCKED at (${newX.toFixed(1)},${newY.toFixed(1)}) from (${this.state.x.toFixed(1)},${this.state.y.toFixed(1)})`);
      this.state.vx = this.state.vy = 0;
    }
    this.state.heading = newHeading;

    return { ...this.state };
  }

  /** Per-wheel speed (mm/s) for spinning the 3 wheel meshes */
  getWheelSpeeds(): [number, number, number] {
    return WHEEL_ANGLES_DEG.map((deg) => {
      const a = deg * DEG2RAD;
      // Omni wheel tangential speed: -sin(a)*vx + cos(a)*vy + R*omega_rad
      return -Math.sin(a) * this.state.vx
           +  Math.cos(a) * this.state.vy
           + (this.robotRadius * 0.001) * (this.state.omega * DEG2RAD);
    }) as [number, number, number];
  }

  private checkCollision(x: number, y: number): boolean {
    for (const obs of this.obstacles) {
      const hw = obs.width  / 2 + this.robotRadius;
      const hh = (obs.type === 'cylinder' ? obs.width : obs.depth) / 2 + this.robotRadius;
      const dx = Math.abs(x - obs.x);
      const dy = Math.abs(y - obs.y);
      if (dx < hw && dy < hh) {
        console.log(`[Physics] COLLISION: pos=(${x.toFixed(1)},${y.toFixed(1)}) vs obs=(${obs.x},${obs.y}) w=${obs.width} d=${obs.depth} h=${obs.height} hw=${hw.toFixed(1)} hh=${hh.toFixed(1)} dx=${dx.toFixed(1)} dy=${dy.toFixed(1)}`);
        return true;
      }
    }
    return false;
  }
}

function ramp(current: number, target: number, accel: number, decel: number): number {
  const diff = target - current;
  if (Math.abs(diff) < 0.5) return target;
  const limit = diff > 0 ? accel : decel;
  return current + Math.sign(diff) * Math.min(Math.abs(diff), limit);
}

function clamp(v: number, min: number, max: number): number {
  return Math.max(min, Math.min(max, v));
}
