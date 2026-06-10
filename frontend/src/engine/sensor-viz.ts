// Sensor visualization — laser arcs, ultrasonic cones, TF-Luna ray

import * as THREE from 'three';

export interface SensorArcMesh {
  arc: THREE.Line;
  hitPoint: THREE.Mesh;
  distance: number;
}

// Laser sensor positions (matching robot-model angles)
const LASER_POSITIONS: Record<string, { x: number; y: number; angle: number }> = {
  laser_left_front: { x: 0.18, y: 0.1, angle: 0.4 },
  laser_left_back: { x: 0.18, y: -0.1, angle: -0.4 },
  laser_right_front: { x: -0.18, y: 0.1, angle: Math.PI - 0.4 },
  laser_right_back: { x: -0.18, y: -0.1, angle: Math.PI + 0.4 },
  laser_back_left: { x: 0.08, y: -0.2, angle: -Math.PI / 2 - 0.3 },
  laser_back_right: { x: -0.08, y: -0.2, angle: -Math.PI / 2 + 0.3 },
};

const ULTRASOUND_POSITIONS: Record<string, { x: number; y: number; angle: number }> = {
  ultra_front_left: { x: 0.08, y: 0.2, angle: 0 },
  ultra_front_right: { x: -0.08, y: 0.2, angle: 0 },
};

const LASER_MAX_RANGE = 1500; // mm
const ULTRASOUND_MAX_RANGE = 4000; // mm
const MM_TO_M = 0.001;

function distanceColor(distMm: number, maxMm: number): number {
  const ratio = distMm / maxMm;
  if (ratio < 0.15) return 0xff2222; // critical — red
  if (ratio < 0.3) return 0xff8800;  // warning — orange
  return 0x00cc66;                    // safe — green
}

export function createLaserArcs(): Map<string, SensorArcMesh> {
  const arcs = new Map<string, SensorArcMesh>();

  Object.entries(LASER_POSITIONS).forEach(([key, pos]) => {
    const arc = createArc(pos.angle, LASER_MAX_RANGE * MM_TO_M, 0x00cc66);
    arc.group.position.set(pos.x, pos.y, 0.12);

    const hitPoint = new THREE.Mesh(
      new THREE.SphereGeometry(0.015, 8, 8),
      new THREE.MeshStandardMaterial({ color: 0x00cc66, emissive: 0x00cc66, emissiveIntensity: 0.5 })
    );

    const group = new THREE.Group();
    group.add(arc.group);
    group.add(hitPoint);

    arcs.set(key, {
      arc: arc.line,
      hitPoint,
      distance: LASER_MAX_RANGE,
    });
  });

  return arcs;
}

export function createUltrasonicCones(): Map<string, SensorArcMesh> {
  const cones = new Map<string, SensorArcMesh>();

  Object.entries(ULTRASOUND_POSITIONS).forEach(([key, pos]) => {
    const cone = createArc(pos.angle, ULTRASOUND_MAX_RANGE * MM_TO_M, 0x4488ff);
    cone.group.position.set(pos.x, pos.y, 0.1);

    const hitPoint = new THREE.Mesh(
      new THREE.SphereGeometry(0.02, 8, 8),
      new THREE.MeshStandardMaterial({ color: 0x4488ff, emissive: 0x4488ff, emissiveIntensity: 0.5 })
    );

    cones.set(key, {
      arc: cone.line,
      hitPoint,
      distance: ULTRASOUND_MAX_RANGE,
    });
  });

  return cones;
}

export function createTfLunaRay(): { line: THREE.Line; hitPoint: THREE.Mesh } {
  const line = createRay(0, LASER_MAX_RANGE * MM_TO_M, 0xff00ff);
  line.position.set(0, 0.28, 0.13);

  const hitPoint = new THREE.Mesh(
    new THREE.SphereGeometry(0.015, 8, 8),
    new THREE.MeshStandardMaterial({ color: 0xff00ff, emissive: 0xff00ff, emissiveIntensity: 0.5 })
  );

  return { line, hitPoint };
}

function createArc(
  baseAngle: number,
  maxRange: number,
  color: number
): { line: THREE.Line; group: THREE.Group } {
  const segments = 16;
  const spread = 0.15; // radians — narrow beam
  const points: THREE.Vector3[] = [];

  for (let i = 0; i <= segments; i++) {
    const t = i / segments;
    const angle = baseAngle - spread / 2 + t * spread;
    points.push(new THREE.Vector3(
      Math.cos(angle) * maxRange,
      Math.sin(angle) * maxRange,
      0
    ));
  }

  const geo = new THREE.BufferGeometry().setFromPoints(points);
  const mat = new THREE.LineBasicMaterial({ color, transparent: true, opacity: 0.4 });
  const line = new THREE.Line(geo, mat);

  const group = new THREE.Group();
  group.add(line);

  return { line, group };
}

function createRay(angle: number, length: number, color: number): THREE.Line {
  const points = [
    new THREE.Vector3(0, 0, 0),
    new THREE.Vector3(Math.cos(angle) * length, Math.sin(angle) * length, 0),
  ];
  const geo = new THREE.BufferGeometry().setFromPoints(points);
  const mat = new THREE.LineBasicMaterial({ color, transparent: true, opacity: 0.5 });
  return new THREE.Line(geo, mat);
}

export function updateSensorArcs(
  sensors: Record<string, number>,
  arcs: Map<string, SensorArcMesh>,
  maxRange: number,
  hitPoints: Map<string, THREE.Mesh>
) {
  Object.entries(sensors).forEach(([key, distMm]) => {
    const arc = arcs.get(key);
    if (!arc) return;

    const distM = distMm * MM_TO_M;
    arc.distance = distMm;

    const color = distanceColor(distMm, maxRange);
    const mat = arc.arc.material as THREE.LineBasicMaterial;
    mat.color.setHex(color);

    // Update hit point position
    const hit = hitPoints.get(key);
    if (hit) {
      const pos = LASER_POSITIONS[key] || ULTRASOUND_POSITIONS[key];
      if (pos) {
        const finalAngle = pos.angle;
        hit.position.set(
          pos.x + Math.cos(finalAngle) * distM,
          pos.y + Math.sin(finalAngle) * distM,
          0.12
        );
      }
      const hitMat = hit.material as THREE.MeshStandardMaterial;
      hitMat.color.setHex(color);
      hitMat.emissive.setHex(color);
    }
  });
}

export function addLaserArcsToScene(arcs: Map<string, SensorArcMesh>, scene: THREE.Scene) {
  arcs.forEach((arc) => {
    scene.add(arc.arc.parent as THREE.Group);
  });
}

export function addUltrasonicConesToScene(cones: Map<string, SensorArcMesh>, scene: THREE.Scene) {
  cones.forEach((cone) => {
    scene.add(cone.arc.parent as THREE.Group);
  });
}
