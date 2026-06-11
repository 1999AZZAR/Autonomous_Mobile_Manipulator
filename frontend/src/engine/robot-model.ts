// Robot 3D model — hexagonal chassis, 3 omni wheels, 6 IR sensors, 2 ultrasonic,
// 3 line sensors, vertical lifter mast with carriage + tilting gripper arm.
//
// Coordinate system (Z-up, matches ROS2):
//   +Y = FRONT   +X = RIGHT   +Z = UP
//   Body: hexagon, vertex pointing in +Y (front is a point, not a flat face)
//   Body occupies z = 0 → 0.12  (12cm tall)

import * as THREE from 'three';
import type { RobotModelParts } from '../types/twin';

// ─── Material factory ────────────────────────────────────────────────────────
function mkMat(
  color: number,
  emissive: number,
  emissiveIntensity: number,
  roughness = 0.55
): THREE.MeshStandardMaterial {
  return new THREE.MeshStandardMaterial({
    color,
    emissive,
    emissiveIntensity,
    roughness,
    metalness: 0,
  });
}

// Hex body vertices (pointy-top, vertex at +Y):
//   angle offset = -π/6  → vertex at 90° = +Y direction
const HEX_R = 0.22; // hexagon circumradius (m)
const BODY_H = 0.12; // body height (m)
const WHEEL_DIST = 0.26; // centre-to-wheel distance (m)

export function createRobotModel(): { group: THREE.Group; parts: RobotModelParts } {
  const group = new THREE.Group();

  // ── 1. BODY ──────────────────────────────────────────────────────────────
  const bodyShape = new THREE.Shape();
  for (let i = 0; i <= 6; i++) {
    const a = (i / 6) * Math.PI * 2 - Math.PI / 6; // -30° offset → vertex at 90° = +Y
    const x = Math.cos(a) * HEX_R;
    const y = Math.sin(a) * HEX_R;
    i === 0 ? bodyShape.moveTo(x, y) : bodyShape.lineTo(x, y);
  }
  const body = new THREE.Mesh(
    new THREE.ExtrudeGeometry(bodyShape, { depth: BODY_H, bevelEnabled: false }),
    mkMat(0x5588ee, 0x1a2a60, 0.45)
  );
  // No rotation — body lies flat in XY, extruded upward in +Z (0 → BODY_H).
  // Front vertex at (0, HEX_R) = +Y direction. Matches wheels/sensors/arm.
  body.castShadow = true;
  group.add(body);

  // ── 2. WHEELS — Motor 2 (FR @ 30°), Motor 3 (FL @ 150°), Motor 4 (Back @ 270°) ──
  const wheels: THREE.Mesh[] = [];
  const WHEEL_ANGLES = [
    Math.PI / 6,          // FR  30°
    (5 * Math.PI) / 6,   // FL  150°
    (3 * Math.PI) / 2,   // Back 270°
  ];

  WHEEL_ANGLES.forEach((angle) => {
    const wx = Math.cos(angle) * WHEEL_DIST;
    const wy = Math.sin(angle) * WHEEL_DIST;

    // Wheel tyre
    const wheel = new THREE.Mesh(
      new THREE.CylinderGeometry(0.045, 0.045, 0.028, 16),
      mkMat(0x445566, 0x1a2233, 0.35, 0.8)
    );
    wheel.position.set(wx, wy, 0.045);
    wheel.rotation.z = angle - Math.PI / 2; // axle points radially outward
    wheel.castShadow = true;
    group.add(wheel);
    wheels.push(wheel);

    // Hub cap accent
    const hub = new THREE.Mesh(
      new THREE.CylinderGeometry(0.018, 0.018, 0.030, 8),
      mkMat(0x7799bb, 0x334455, 0.3)
    );
    hub.position.set(wx, wy, 0.045);
    hub.rotation.z = angle - Math.PI / 2;
    group.add(hub);

    // Mount bracket from body edge to wheel
    const bx = Math.cos(angle) * (HEX_R - 0.02);
    const by = Math.sin(angle) * (HEX_R - 0.02);
    const bracket = new THREE.Mesh(
      new THREE.BoxGeometry(0.022, 0.022, 0.075),
      mkMat(0x334455, 0x112233, 0.25)
    );
    bracket.position.set(
      bx + (wx - bx) * 0.5,
      by + (wy - by) * 0.5,
      0.055
    );
    group.add(bracket);
  });

  // ── 3. HEADING ARROW (front indicator, bright red) ───────────────────────
  const arrowS = new THREE.Shape();
  arrowS.moveTo(0, 0.065); arrowS.lineTo(-0.02, 0.015); arrowS.lineTo(0.02, 0.015);
  arrowS.closePath();
  const headingArrow = new THREE.Mesh(
    new THREE.ExtrudeGeometry(arrowS, { depth: 0.006, bevelEnabled: false }),
    mkMat(0xff3333, 0xff3333, 0.7)
  );
  headingArrow.position.set(0, 0.14, 0.125);
  group.add(headingArrow);

  // ── 4. IR DISTANCE SENSORS (6 Sharp GP2Y0A02YK0F) ───────────────────────
  // Hexagon flat face centres (body radius at mid-edge ≈ 0.22 * cos(30°) = 0.190):
  //   Right face (A2/A9):  x=+0.190, y=0        normals face +X
  //   Left  face (A0/A1):  x=-0.190, y=0        normals face -X
  //   Back-right (A10):    back-right diagonal   normals face 330° (= +X, -Y diagonal)
  //   Back-left  (A11):    back-left  diagonal   normals face 210° (= -X, -Y diagonal)
  //
  //  Two sensors per side, offset ±40mm along the face.
  const laserSensors: THREE.Mesh[] = [];
  const FACE_R = HEX_R * Math.cos(Math.PI / 6); // ≈ 0.190 — mid-edge apothem

  // [x, y, rotation-z] — rotation-z aligns sensor to face outward from that face
  const IR_DEFS: [number, number, number][] = [
    // IR Right 1 (A2)  — right face, front half
    [ FACE_R,  0.045, 0],
    // IR Right 2 (A9)  — right face, back half
    [ FACE_R, -0.045, 0],
    // IR Left 1 (A0)   — left face, front half
    [-FACE_R,  0.045, Math.PI],
    // IR Left 2 (A1)   — left face, back half
    [-FACE_R, -0.045, Math.PI],
    // IR Back 1 (A10)  — back-right face
    [ 0.09, -0.175, -Math.PI * 2 / 3],
    // IR Back 2 (A11)  — back-left face
    [-0.09, -0.175,  Math.PI * 2 / 3],
  ];

  IR_DEFS.forEach(([x, y, rz]) => {
    const s = new THREE.Mesh(
      new THREE.BoxGeometry(0.028, 0.016, 0.014),
      mkMat(0x00ff88, 0x00ff88, 0.6)
    );
    s.position.set(x, y, 0.085);
    s.rotation.z = rz;
    group.add(s);
    laserSensors.push(s);
  });

  // ── 5. ULTRASONIC SENSORS — HC-SR04, front-left & front-right ───────────
  // Placed on the two front diagonal faces.
  // Front-right face midpoint: (0.095, 0.165). Face normal at 30° from +X.
  // Front-left  face midpoint: (-0.095, 0.165). Face normal at 150°.
  const ultraSensors: THREE.Mesh[] = [];
  [
    { x:  0.095, y: 0.165, rz: -Math.PI / 6 },  // Ultrasonic FR (D24/D25)
    { x: -0.095, y: 0.165, rz:  Math.PI / 6 },  // Ultrasonic FL (D22/D23)
  ].forEach(({ x, y, rz }) => {
    const s = new THREE.Mesh(
      new THREE.CylinderGeometry(0.013, 0.013, 0.020, 10),
      mkMat(0x44ddff, 0x44ddff, 0.55)
    );
    s.position.set(x, y, 0.085);
    s.rotation.x = Math.PI / 2;
    s.rotation.z = rz;
    group.add(s);
    ultraSensors.push(s);
  });

  // ── 6. LINE SENSORS (A6/A7/A8) — 3 in a row at the front underside ──────
  const lineSensors: THREE.Mesh[] = [];
  [-0.048, 0, 0.048].forEach((lx) => {
    const s = new THREE.Mesh(
      new THREE.CircleGeometry(0.010, 8),
      mkMat(0xffcc00, 0xffcc00, 0.7)
    );
    s.position.set(lx, 0.15, 0.002);
    s.rotation.x = -Math.PI / 2;
    group.add(s);
    lineSensors.push(s);
  });

  // ── 7. LIFTER MASTS — 2 tall vertical orange poles at the front ─────────
  // They start at z=0 (chassis) and rise to z=0.40 (well above body top at 0.12).
  // Placed at y=0.19 (just inside the front vertex at y=0.22) and x=±0.052.
  const MAST_H    = 0.42;
  const MAST_BOT  = 0.00;
  const MAST_TOP  = MAST_BOT + MAST_H;
  const MAST_CY   = MAST_BOT + MAST_H / 2; // z centre of mast cylinder
  const MAST_Y    = 0.19;                   // front placement
  const MAST_X    = 0.052;                  // half-gap between the two masts
  const mastMat   = mkMat(0xff9900, 0x883300, 0.6, 0.4);

  [-MAST_X, MAST_X].forEach((mx) => {
    const mast = new THREE.Mesh(
      new THREE.CylinderGeometry(0.013, 0.013, MAST_H, 8),
      mastMat
    );
    mast.position.set(mx, MAST_Y, MAST_CY);
    mast.rotation.x = Math.PI / 2; // cylinder axis → Z (vertical)
    mast.castShadow = true;
    group.add(mast);
  });

  // Top cross-beam connecting the two masts
  const crossBeam = new THREE.Mesh(
    new THREE.BoxGeometry(MAST_X * 2 + 0.035, 0.022, 0.022),
    mastMat
  );
  crossBeam.position.set(0, MAST_Y, MAST_TOP);
  group.add(crossBeam);

  // ── 8. CARRIAGE — slides vertically on masts, does NOT tilt ─────────────
  // Initial z driven by updateLifter(50) → z = 0.18 + 0.5×0.18 = 0.27
  const carriage = new THREE.Mesh(
    new THREE.BoxGeometry(MAST_X * 2 + 0.030, 0.028, 0.060),
    mkMat(0xffcc33, 0x885500, 0.55, 0.45)
  );
  carriage.position.set(0, MAST_Y, 0.27); // initial position (lifterHeight 50%)
  carriage.castShadow = true;
  group.add(carriage);

  // ── 9. GRIPPER ASSEMBLY — pivot at carriage height, arm extends in +Z ───
  //  Tilt formula: rotation.x = -(angle × π/180)
  //    0°  → arm points up (+Z)
  //    90° → arm points forward (+Y)
  //    180°→ arm points down (-Z, reaching floor)
  const gripperAssembly = new THREE.Group();
  gripperAssembly.position.set(0, MAST_Y + 0.020, 0.27);

  // Tilt servo housing (the pivot block)
  const servoMesh = new THREE.Mesh(
    new THREE.BoxGeometry(0.028, 0.020, 0.028),
    mkMat(0xff9900, 0x883300, 0.5, 0.4)
  );
  gripperAssembly.add(servoMesh);

  // Arm column — extends in +Z from the servo pivot
  const armColumn = new THREE.Mesh(
    new THREE.BoxGeometry(0.016, 0.016, 0.115),
    mkMat(0xff9900, 0x883300, 0.5, 0.4)
  );
  armColumn.position.set(0, 0, 0.060); // centre at z=0.060 → spans z=0.002..0.118
  armColumn.castShadow = true;
  gripperAssembly.add(armColumn);

  // Gripper base crossbar at arm tip
  const gripperBase = new THREE.Mesh(
    new THREE.BoxGeometry(0.068, 0.018, 0.014),
    mkMat(0xddeeff, 0x334455, 0.3)
  );
  gripperBase.position.set(0, 0, 0.122);
  gripperBase.castShadow = true;
  gripperAssembly.add(gripperBase);

  // Fingers (open/close in ±X, extend further in +Z)
  const fingerGeo = new THREE.BoxGeometry(0.010, 0.015, 0.045);
  const fingerMat = mkMat(0xddeeff, 0x334455, 0.3);

  const leftFinger = new THREE.Mesh(fingerGeo, fingerMat);
  leftFinger.position.set(-0.024, 0, 0.146);
  leftFinger.castShadow = true;
  gripperAssembly.add(leftFinger);

  const rightFinger = new THREE.Mesh(fingerGeo, fingerMat);
  rightFinger.position.set(0.024, 0, 0.146);
  rightFinger.castShadow = true;
  gripperAssembly.add(rightFinger);

  // Camera (on servo housing front, dark box facing +Y)
  const camMesh = new THREE.Mesh(
    new THREE.BoxGeometry(0.026, 0.014, 0.019),
    new THREE.MeshStandardMaterial({ color: 0x223344, roughness: 0.6 })
  );
  camMesh.position.set(-0.018, 0.018, 0.002);
  gripperAssembly.add(camMesh);

  // TF-Luna (cyan box beside camera)
  const tfMesh = new THREE.Mesh(
    new THREE.BoxGeometry(0.013, 0.011, 0.013),
    mkMat(0x44bbdd, 0x22aacc, 0.4)
  );
  tfMesh.position.set(0.018, 0.018, 0.002);
  gripperAssembly.add(tfMesh);

  group.add(gripperAssembly);

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

// ─── Actuator update functions ────────────────────────────────────────────────

export function updateGripper(open: boolean, parts: RobotModelParts) {
  const gap = open ? 0.028 : 0.006;
  parts.gripper.left.position.x  = -gap;
  parts.gripper.right.position.x =  gap;
}

export function updateTiltServo(angle: number, parts: RobotModelParts) {
  // 0° = arm up (+Z vertical), 90° = arm forward (+Y), 180° = arm down (-Z)
  parts.gripperAssembly.rotation.x = -(angle * Math.PI) / 180;
}

export function updateLifter(height: number, parts: RobotModelParts) {
  const h = Math.max(0, Math.min(100, height));
  const z = 0.18 + (h / 100) * 0.18; // range z=0.18 (bottom) → z=0.36 (top)
  parts.carriage.position.z       = z;
  parts.gripperAssembly.position.z = z;
}

export function updateLineSensors(
  readings: { line_left: number; line_center: number; line_right: number },
  parts: RobotModelParts
) {
  const threshold = 512;
  [readings.line_left, readings.line_center, readings.line_right].forEach((v, i) => {
    const m = parts.lineSensors[i].material as THREE.MeshStandardMaterial;
    const on = v < threshold;
    m.emissiveIntensity = on ? 1.0 : 0.5;
    m.color.setHex(on ? 0xffffff : 0xffcc00);
  });
}

export function updateWheelRotation(speed: number, parts: RobotModelParts) {
  parts.wheels.forEach((w) => { w.rotation.y += speed * 0.012; });
}
