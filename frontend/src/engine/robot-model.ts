// Robot mesh builder — hexagonal chassis, 3 omni wheels, sensors, gripper+lifter assembly

import * as THREE from 'three';
import type { RobotModelParts } from '../types/twin';

const COLORS = {
  body:       0x6e9ef5,   // bright cornflower blue
  bodyAccent: 0x99bbff,   // lighter blue
  wheel:      0x7788aa,   // medium slate
  gripper:    0xe8eeff,   // near-white
  laser:      0x00ff88,
  ultrasonic: 0x44ddff,
  line:       0xffcc00,
  heading:    0xff4444,
  lifter:     0xffaa33,   // bright orange — masts/arm stand out clearly
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
  const bodyMat = new THREE.MeshStandardMaterial({
    color: COLORS.body,
    roughness: 0.5,
    metalness: 0,
    emissive: 0x1a2a60,
    emissiveIntensity: 0.4,
  });
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
  // FR at 30°, FL at 150°, Back at 270° — matches Motor2/Motor3/Motor4 physical positions
  const wheelAngles = [Math.PI / 6, (5 * Math.PI) / 6, (3 * Math.PI) / 2];

  wheelAngles.forEach((angle) => {
    const geo = new THREE.CylinderGeometry(wheelRadius, wheelRadius, wheelWidth, 16);
    const mat = new THREE.MeshStandardMaterial({ color: COLORS.wheel, roughness: 0.7, metalness: 0, emissive: 0x223344, emissiveIntensity: 0.3 });
    const wheel = new THREE.Mesh(geo, mat);
    const wx = Math.cos(angle) * wheelDist;
    const wy = Math.sin(angle) * wheelDist;
    wheel.position.set(wx, wy, wheelRadius);
    wheel.rotation.z = angle - Math.PI / 2; // axle points radially outward
    wheel.castShadow = true;
    group.add(wheel);
    wheels.push(wheel);

    const bracketGeo = new THREE.BoxGeometry(0.01, 0.04, 0.06);
    const bracketMat = new THREE.MeshStandardMaterial({ color: COLORS.lifter, roughness: 0.6, metalness: 0, emissive: 0x334455, emissiveIntensity: 0.3 });
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

  // === LIFTER MASTS — tall static vertical structure on front face ===
  const mastHeight = 0.30;
  const mastMat = new THREE.MeshStandardMaterial({
    color: COLORS.lifter, roughness: 0.5, metalness: 0,
    emissive: 0x884400, emissiveIntensity: 0.45,
  });
  [-0.042, 0.042].forEach((rx) => {
    const mast = new THREE.Mesh(
      new THREE.CylinderGeometry(0.012, 0.012, mastHeight, 8),  // 12mm — clearly visible
      mastMat
    );
    mast.position.set(rx, 0.22, 0.04 + mastHeight / 2); // center z=0.19
    mast.rotation.x = Math.PI / 2; // vertical along Z
    mast.castShadow = true;
    group.add(mast);
  });
  // Top cross-beam
  const crossBeam = new THREE.Mesh(
    new THREE.BoxGeometry(0.115, 0.022, 0.022),
    mastMat
  );
  crossBeam.position.set(0, 0.22, 0.04 + mastHeight);
  group.add(crossBeam);

  // === CARRIAGE — slides up/down on masts, does NOT tilt ===
  const carriageMat = new THREE.MeshStandardMaterial({
    color: COLORS.bodyAccent, roughness: 0.5, metalness: 0,
    emissive: 0x884400, emissiveIntensity: 0.45,
  });
  const carriage = new THREE.Mesh(
    new THREE.BoxGeometry(0.11, 0.03, 0.055),
    carriageMat
  );
  carriage.position.set(0, 0.22, 0.20); // initial (lifterHeight=50 → z=0.20)
  carriage.castShadow = true;
  group.add(carriage);

  // === GRIPPER ASSEMBLY — arm in +Z, tilts around X at carriage height ===
  const gripperAssembly = new THREE.Group();
  gripperAssembly.position.set(0, 0.235, 0.20);
  gripperAssembly.rotation.x = 0; // 0° = arm straight up (+Z), matches tiltAngle=0 default

  // Tilt servo housing (the pivot point)
  const servoGeo = new THREE.BoxGeometry(0.026, 0.018, 0.026);
  const servoMat = new THREE.MeshStandardMaterial({
    color: COLORS.lifter, roughness: 0.4, metalness: 0,
    emissive: 0x884400, emissiveIntensity: 0.4,
  });
  const servoMesh = new THREE.Mesh(servoGeo, servoMat);
  gripperAssembly.add(servoMesh);

  // Arm column — extends in +Z (upward) from the pivot
  const armColMat = new THREE.MeshStandardMaterial({
    color: COLORS.lifter, roughness: 0.5, metalness: 0,
    emissive: 0x884400, emissiveIntensity: 0.4,
  });
  const armColumn = new THREE.Mesh(
    new THREE.BoxGeometry(0.014, 0.014, 0.10),
    armColMat
  );
  armColumn.position.set(0, 0, 0.055); // spans local z=0.005 to z=0.105
  armColumn.castShadow = true;
  gripperAssembly.add(armColumn);

  // Gripper base crossbar at tip of arm column
  const gripperBaseMat = new THREE.MeshStandardMaterial({
    color: COLORS.gripper, roughness: 0.4, metalness: 0,
    emissive: 0x445577, emissiveIntensity: 0.3,
  });
  const gripperBase = new THREE.Mesh(
    new THREE.BoxGeometry(0.065, 0.018, 0.013),
    gripperBaseMat
  );
  gripperBase.position.set(0, 0, 0.108);
  gripperBase.castShadow = true;
  gripperAssembly.add(gripperBase);

  // Gripper fingers — extend further in +Z, open/close in ±X
  const fingerMat = new THREE.MeshStandardMaterial({
    color: COLORS.gripper, roughness: 0.4, metalness: 0,
    emissive: 0x445566, emissiveIntensity: 0.25,
  });
  const leftFinger = new THREE.Mesh(
    new THREE.BoxGeometry(0.009, 0.016, 0.042),
    fingerMat
  );
  leftFinger.position.set(-0.022, 0, 0.132);
  leftFinger.castShadow = true;
  gripperAssembly.add(leftFinger);

  const rightFinger = new THREE.Mesh(
    new THREE.BoxGeometry(0.009, 0.016, 0.042),
    fingerMat
  );
  rightFinger.position.set(0.022, 0, 0.132);
  rightFinger.castShadow = true;
  gripperAssembly.add(rightFinger);

  // Camera on servo housing (points forward +Y)
  const camera = new THREE.Mesh(
    new THREE.BoxGeometry(0.024, 0.014, 0.018),
    new THREE.MeshStandardMaterial({ color: 0x223344 })
  );
  camera.position.set(-0.02, 0.018, 0);
  gripperAssembly.add(camera);

  // TF-Luna beside camera
  const tfLuna = new THREE.Mesh(
    new THREE.BoxGeometry(0.012, 0.010, 0.012),
    new THREE.MeshStandardMaterial({ color: 0x44aacc, emissive: 0x22aacc, emissiveIntensity: 0.35 })
  );
  tfLuna.position.set(0.02, 0.018, 0);
  gripperAssembly.add(tfLuna);

  group.add(gripperAssembly);

  group.position.set(0, 0, 0);

  return {
    group,
    parts: {
      body,
      wheels,
      gripper: { left: leftFinger, right: rightFinger },
      gripperAssembly,
      carriage,
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
  // Arm extends in +Z. rotation.x = 0 → vertical up (0°), -π/2 → forward (90°), -π → down (180°)
  const rad = -(angle * Math.PI) / 180;
  parts.gripperAssembly.rotation.x = rad;
}

export function updateLifter(height: number, parts: RobotModelParts) {
  const clamped = Math.max(0, Math.min(100, height));
  const z = 0.10 + (clamped / 100) * 0.20; // travels z=0.10 → z=0.30 along the mast
  parts.carriage.position.z = z;           // carriage stays upright
  parts.gripperAssembly.position.z = z;    // arm tilts from same height
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
