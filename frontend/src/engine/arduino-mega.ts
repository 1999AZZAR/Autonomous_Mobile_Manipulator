// Simulated Arduino Mega — mirrors omni_motor_control.ino firmware.
//
// Real hardware: RPi sends (vx, vy, omega) via serial → Arduino reads 6 IR +
// 2 ultrasonic sensors → Virtual Force Field → perimeter safety → PID motors.
//
// This module replicates that flow in mm/s (physics-engine units).
// Sensor raycasts use wouldCollide() which adds robot radius, so we subtract
// robotRadius to recover the SURFACE distance used by the real VFF.
//
// Coordinate system: +vx = right, +vy = forward (matches PhysicsEngine).

export interface VffResult {
  vx: number;
  vy: number;
  omega: number;
  braking: boolean;
  forceMagnitude: number;
  sensorReadings: number[];   // surface distances (mm)
}

const ROBOT_RADIUS = 230; // mm — must match PhysicsEngine.robotRadius

// ---- Sensor layout (matches real robot) --------------------------------
// dists are SURFACE distances (not robot-center).

interface SensorDef {
  dirX: number;
  dirY: number;
  maxRange: number;      // mm (surface)
  influenceDist: number; // mm (surface) — VFF activates within this range
  criticalDist: number;  // mm (surface) — emergency brake
  scaleFactor: number;
}

const SENSORS: SensorDef[] = [
  // Left IR ×2
  { dirX: -1, dirY:  0, maxRange: 1500, influenceDist: 400, criticalDist: 120,  scaleFactor: 2.0 },
  { dirX: -1, dirY:  0, maxRange: 1500, influenceDist: 400, criticalDist: 120,  scaleFactor: 2.0 },
  // Right IR ×2
  { dirX:  1, dirY:  0, maxRange: 1500, influenceDist: 400, criticalDist: 120,  scaleFactor: 2.0 },
  { dirX:  1, dirY:  0, maxRange: 1500, influenceDist: 400, criticalDist: 120,  scaleFactor: 2.0 },
  // Back IR ×2
  { dirX:  0, dirY: -1, maxRange: 1500, influenceDist: 400, criticalDist: 120,  scaleFactor: 2.0 },
  { dirX:  0, dirY: -1, maxRange: 1500, influenceDist: 400, criticalDist: 120,  scaleFactor: 2.0 },
  // Ultrasonic front-left
  { dirX: -0.3, dirY: 1, maxRange: 5000, influenceDist: 500, criticalDist: 180, scaleFactor: 3.0 },
  // Ultrasonic front-right
  { dirX:  0.3, dirY: 1, maxRange: 5000, influenceDist: 500, criticalDist: 180, scaleFactor: 3.0 },
  // Ultrasonic front-center
  { dirX:  0,   dirY: 1, maxRange: 5000, influenceDist: 500, criticalDist: 180, scaleFactor: 3.0 },
];

const RAY_STEP = 15; // mm (robot-center space)

// ---- Public API -------------------------------------------------------

export function processArduinoMega(
  vx: number, vy: number, omega: number,
  robotX: number, robotY: number,
  heading: number,
  wouldCollide: (x: number, y: number) => boolean,
): VffResult {
  const readings: number[] = [];
  let avoidVx = 0;
  let avoidVy = 0;
  let maxProximity = 0;
  let braking = false;

  const desiredSpeed = Math.sqrt(vx * vx + vy * vy) || 200;

  // Rotate robot-frame sensor directions to world frame for raycasting
  const hRad = heading * (Math.PI / 180);
  const cosH = Math.cos(hRad);
  const sinH = Math.sin(hRad);

  for (const s of SENSORS) {
    // Rotate sensor direction from robot frame to world frame
    const worldDirX = s.dirX * cosH + s.dirY * sinH;
    const worldDirY = -s.dirX * sinH + s.dirY * cosH;

    // Raycast in world space
    const centerDist = raycast(worldDirX, worldDirY, s.maxRange + ROBOT_RADIUS, robotX, robotY, wouldCollide);
    // Convert to surface distance (what the real sensor would read)
    const surfaceDist = Math.max(0, centerDist - ROBOT_RADIUS);
    readings.push(surfaceDist);

    // Perimeter safety
    if (surfaceDist < s.criticalDist) {
      braking = true;
      continue;
    }

    if (surfaceDist >= s.influenceDist) continue;

    // Proximity: 0 at influence boundary → 1 at sensor face
    const proximity = 1 - surfaceDist / s.influenceDist;

    // Avoidance velocity: pushes away from obstacle, in ROBOT frame
    const avoidSpeed = s.scaleFactor * proximity * proximity * desiredSpeed * 0.5;
    avoidVx += -s.dirX * avoidSpeed;
    avoidVy += -s.dirY * avoidSpeed;

    if (proximity > maxProximity) maxProximity = proximity;
  }

  if (braking) {
    return { vx: 0, vy: 0, omega: 0, braking: true, forceMagnitude: maxProximity, sensorReadings: readings };
  }

  // Cap total avoidance so goal-seeking always has at least 20% influence
  const avoidMag = Math.sqrt(avoidVx * avoidVx + avoidVy * avoidVy);
  const maxAvoid = desiredSpeed * 0.8;
  if (avoidMag > maxAvoid && avoidMag > 0) {
    avoidVx = (avoidVx / avoidMag) * maxAvoid;
    avoidVy = (avoidVy / avoidMag) * maxAvoid;
  }

  const vxMod = clamp(vx + avoidVx, -desiredSpeed * 1.2, desiredSpeed * 1.2);
  const vyMod = clamp(vy + avoidVy, -desiredSpeed * 1.2, desiredSpeed * 1.2);

  return { vx: vxMod, vy: vyMod, omega, braking: false, forceMagnitude: maxProximity, sensorReadings: readings };
}

// ---- Internal ---------------------------------------------------------

function raycast(
  dirX: number, dirY: number, maxRange: number,
  ox: number, oy: number,
  wouldCollide: (x: number, y: number) => boolean,
): number {
  const len = Math.sqrt(dirX * dirX + dirY * dirY) || 1;
  const ux = dirX / len;
  const uy = dirY / len;
  for (let d = RAY_STEP; d <= maxRange; d += RAY_STEP) {
    if (wouldCollide(ox + ux * d, oy + uy * d)) return d;
  }
  return maxRange;
}

function clamp(v: number, lo: number, hi: number): number {
  return Math.max(lo, Math.min(hi, v));
}
