// Robot mesh builder — hexagonal chassis, 3 omni wheels, sensors, gripper+lifter assembly

import * as THREE from 'three';
import type { RobotModelParts } from '../types/twin';

const COLORS = {
  body: 0x3a3a5c,
  bodyAccent: 0x4a4a6c,
  wheel: 0x222222,
  gripper: 0x888888,
  laser: 0x00cc66,
  ultrasonic: 0x4488ff,
  line: 0xffaa00,
  heading: 0xff3333,
  lifter: 0x555577,
};

export function createRobotModel(): { group: THREE.Group; parts: RobotModelParts } {
  const group = new THREE.Group();

  // === BODY — hexagonal prism ===
  const bodyShape = new THREE.Shape();
  const sides = 6;
  const radius = 0.22;
  for (let i = 0; i <= sides; i++) {
    const angle = (i / sides) * Math.PI * 2 - Math.PI / 6;
    const x = Math.cos(angle) * radius;
    const y = Math.sin(angle) * radius;
    if (i === 0) bodyShape.moveTo(x, y);
    else bodyShape.lineTo(x, y);
  }
  const bodyGeo = new THREE.ExtrudeGeometry(bodyShape, { depth: 0.12, bevelEnabled: false });
  const bodyMat = new THREE.MeshStandardMaterial({ color: COLORS.body, roughness: 0.6 });
  const body = new THREE.Mesh(bodyGeo, bodyMat);
  body.rotation.x = -Math.PI / 2;
  body.position.z = 0.06;
  body.castShadow = true;
  group.add(body);

  // === WHEELS — 3 omni wheels at 120° spacing ===
  const wheels: THREE.Mesh[] = [];
  const wheelRadius = 0.04;
  const wheelWidth = 0.03;
  const wheelDist = 0.25;
  const wheelAngles = [0, (2 * Math.PI) / 3, (4 * Math.PI) / 3];

  wheelAngles.forEach((angle) => {
    const geo = new THREE.CylinderGeometry(wheelRadius, wheelRadius, wheelWidth, 16);
    const mat = new THREE.MeshStandardMaterial({ color: COLORS.wheel, roughness: 0.8 });
    const wheel = new THREE.Mesh(geo, mat);
    const wx = Math.cos(angle) * wheelDist;
    const wy = Math.sin(angle) * wheelDist;
    wheel.position.set(wx, wy, wheelRadius);
    wheel.rotation.z = Math.PI / 2;
    wheel.castShadow = true;
    group.add(wheel);
    wheels.push(wheel);

    const bracketGeo = new THREE.BoxGeometry(0.01, 0.04, 0.06);
    const bracketMat = new THREE.MeshStandardMaterial({ color: COLORS.lifter });
    const bracket = new THREE.Mesh(bracketGeo, bracketMat);
    bracket.position.set(wx * 0.85, wy * 0.85, 0.06);
    group.add(bracket);
  });

  // === HEADING ARROW ===
  const arrowShape = new THREE.Shape();
  arrowShape.moveTo(0, 0.08);
  arrowShape.lineTo(-0.025, 0.02);
  arrowShape.lineTo(0.025, 0.02);
  arrowShape.closePath();
  const arrowGeo = new THREE.ExtrudeGeometry(arrowShape, { depth: 0.005, bevelEnabled: false });
  const arrowMat = new THREE.MeshStandardMaterial({ color: COLORS.heading, emissive: COLORS.heading, emissiveIntensity: 0.3 });
  const headingArrow = new THREE.Mesh(arrowGeo, arrowMat);
  headingArrow.position.set(0, 0.18, 0.13);
  group.add(headingArrow);

  // === LASER SENSORS — 6 positions ===
  const laserSensors: THREE.Mesh[] = [];
  const laserPositions = [
    { x: 0.18, y: 0.1, angle: 0.4 },
    { x: 0.18, y: -0.1, angle: -0.4 },
    { x: -0.18, y: 0.1, angle: Math.PI - 0.4 },
    { x: -0.18, y: -0.1, angle: Math.PI + 0.4 },
    { x: 0.08, y: -0.2, angle: -Math.PI / 2 - 0.3 },
    { x: -0.08, y: -0.2, angle: -Math.PI / 2 + 0.3 },
  ];

  laserPositions.forEach((lp) => {
    const geo = new THREE.BoxGeometry(0.03, 0.02, 0.015);
    const mat = new THREE.MeshStandardMaterial({ color: COLORS.laser, emissive: COLORS.laser, emissiveIntensity: 0.2 });
    const sensor = new THREE.Mesh(geo, mat);
    sensor.position.set(lp.x, lp.y, 0.12);
    sensor.rotation.z = lp.angle;
    sensor.castShadow = true;
    group.add(sensor);
    laserSensors.push(sensor);
  });

  // === ULTRASONIC SENSORS — 2 front ===
  const ultraSensors: THREE.Mesh[] = [];
  [
    { x: 0.08, y: 0.2 },
    { x: -0.08, y: 0.2 },
  ].forEach((up) => {
    const geo = new THREE.CylinderGeometry(0.015, 0.015, 0.02, 8);
    const mat = new THREE.MeshStandardMaterial({ color: COLORS.ultrasonic, emissive: COLORS.ultrasonic, emissiveIntensity: 0.2 });
    const sensor = new THREE.Mesh(geo, mat);
    sensor.position.set(up.x, up.y, 0.1);
    sensor.rotation.x = Math.PI / 2;
    sensor.castShadow = true;
    group.add(sensor);
    ultraSensors.push(sensor);
  });

  // === LINE SENSORS — 3 bottom dots ===
  const lineSensors: THREE.Mesh[] = [];
  [-0.06, 0, 0.06].forEach((lx) => {
    const geo = new THREE.CircleGeometry(0.008, 8);
    const mat = new THREE.MeshStandardMaterial({ color: COLORS.line, emissive: COLORS.line, emissiveIntensity: 0.4 });
    const sensor = new THREE.Mesh(geo, mat);
    sensor.position.set(lx, 0.15, 0.001);
    sensor.rotation.x = -Math.PI / 2;
    group.add(sensor);
    lineSensors.push(sensor);
  });

  // === GRIPPER ASSEMBLY — tilt servo + gripper + camera + TF-Luna, moves up/down via lifter ===
  const gripperAssembly = new THREE.Group();
  gripperAssembly.position.set(0, 0.22, 0.12);

  // Tilt servo housing
  const servoGeo = new THREE.BoxGeometry(0.04, 0.03, 0.03);
  const servoMat = new THREE.MeshStandardMaterial({ color: COLORS.bodyAccent });
  const servoMesh = new THREE.Mesh(servoGeo, servoMat);
  gripperAssembly.add(servoMesh);

  // Gripper base
  const gripperBase = new THREE.Mesh(
    new THREE.BoxGeometry(0.06, 0.02, 0.03),
    new THREE.MeshStandardMaterial({ color: COLORS.gripper })
  );
  gripperBase.position.set(0, 0.03, 0);
  gripperBase.castShadow = true;
  gripperAssembly.add(gripperBase);

  // Gripper fingers
  const fingerGeo = new THREE.BoxGeometry(0.008, 0.04, 0.02);
  const fingerMat = new THREE.MeshStandardMaterial({ color: COLORS.gripper });
  const leftFinger = new THREE.Mesh(fingerGeo, fingerMat);
  leftFinger.position.set(-0.02, 0.06, 0);
  leftFinger.castShadow = true;
  gripperAssembly.add(leftFinger);

  const rightFinger = new THREE.Mesh(fingerGeo, fingerMat);
  rightFinger.position.set(0.02, 0.06, 0);
  rightFinger.castShadow = true;
  gripperAssembly.add(rightFinger);

  // Camera (on gripper)
  const camGeo = new THREE.BoxGeometry(0.03, 0.02, 0.02);
  const camMat = new THREE.MeshStandardMaterial({ color: 0x222222 });
  const camera = new THREE.Mesh(camGeo, camMat);
  camera.position.set(0, -0.02, 0.02);
  gripperAssembly.add(camera);

  // TF-Luna (on gripper)
  const tfGeo = new THREE.BoxGeometry(0.015, 0.01, 0.01);
  const tfMat = new THREE.MeshStandardMaterial({ color: 0x44aacc });
  const tfLuna = new THREE.Mesh(tfGeo, tfMat);
  tfLuna.position.set(0.04, -0.02, 0.02);
  gripperAssembly.add(tfLuna);

  group.add(gripperAssembly);

  // === LIFTER RAILS — vertical guides for gripper assembly ===
  const railGeo = new THREE.CylinderGeometry(0.005, 0.005, 0.1, 6);
  const railMat = new THREE.MeshStandardMaterial({ color: COLORS.lifter });
  [-0.035, 0.035].forEach((rx) => {
    const rail = new THREE.Mesh(railGeo, railMat);
    rail.position.set(rx, 0.22, 0.17);
    group.add(rail);
  });

  group.position.set(0, 0, 0);

  return {
    group,
    parts: {
      body,
      wheels,
      gripper: { left: leftFinger, right: rightFinger },
      gripperAssembly,
      tiltServo: servoMesh as any,
      laserSensors,
      ultraSensors,
      lineSensors,
      headingArrow,
    },
  };
}

export function updateGripper(open: boolean, parts: RobotModelParts) {
  const gap = open ? 0.025 : 0.005;
  parts.gripper.left.position.x = -gap;
  parts.gripper.right.position.x = gap;
}

export function updateTiltServo(angle: number, parts: RobotModelParts) {
  const rad = ((angle - 90) * Math.PI) / 180;
  parts.gripperAssembly.rotation.x = rad;
}

export function updateLifter(height: number, parts: RobotModelParts) {
  const clamped = Math.max(0, Math.min(100, height));
  const zOffset = (clamped / 100) * 0.08;
  parts.gripperAssembly.position.z = 0.12 + zOffset;
}

export function updateLineSensors(readings: { line_left: number; line_center: number; line_right: number }, parts: RobotModelParts) {
  const threshold = 512;
  const values = [readings.line_left, readings.line_center, readings.line_right];
  parts.lineSensors.forEach((sensor, i) => {
    const mat = sensor.material as THREE.MeshStandardMaterial;
    mat.emissiveIntensity = values[i] < threshold ? 0.8 : 0.2;
    mat.color.setHex(values[i] < threshold ? 0xffcc00 : COLORS.line);
  });
}

export function updateWheelRotation(speed: number, parts: RobotModelParts) {
  parts.wheels.forEach((wheel) => {
    wheel.rotation.y += speed * 0.01;
  });
}
