// Camera frustum wireframe + sensor distance labels

import * as THREE from 'three';

const FRUSTUM_COLOR = 0xff6b6b;
const FRUSTUM_LENGTH = 0.8;
const FRUSTUM_FOV = 60; // degrees
const FRUSTUM_NEAR = 0.05;
const FRUSTUM_FAR = FRUSTUM_LENGTH;

export function createCameraFrustum(): THREE.Group {
  const group = new THREE.Group();

  // FOV cone wireframe
  const fovRad = (FRUSTUM_FOV * Math.PI) / 180;
  const halfFov = fovRad / 2;
  const nearR = Math.tan(halfFov) * FRUSTUM_NEAR;
  const farR = Math.tan(halfFov) * FRUSTUM_FAR;

  // Near plane rectangle
  const nearPoints = [
    new THREE.Vector3(-nearR, FRUSTUM_NEAR, -nearR),
    new THREE.Vector3(nearR, FRUSTUM_NEAR, -nearR),
    new THREE.Vector3(nearR, FRUSTUM_NEAR, nearR),
    new THREE.Vector3(-nearR, FRUSTUM_NEAR, nearR),
    new THREE.Vector3(-nearR, FRUSTUM_NEAR, -nearR),
  ];
  const nearGeo = new THREE.BufferGeometry().setFromPoints(nearPoints);
  const nearMat = new THREE.LineBasicMaterial({ color: FRUSTUM_COLOR, transparent: true, opacity: 0.5 });
  const nearLine = new THREE.Line(nearGeo, nearMat);
  group.add(nearLine);

  // Far plane rectangle
  const farPoints = [
    new THREE.Vector3(-farR, FRUSTUM_FAR, -farR),
    new THREE.Vector3(farR, FRUSTUM_FAR, -farR),
    new THREE.Vector3(farR, FRUSTUM_FAR, farR),
    new THREE.Vector3(-farR, FRUSTUM_FAR, farR),
    new THREE.Vector3(-farR, FRUSTUM_FAR, -farR),
  ];
  const farGeo = new THREE.BufferGeometry().setFromPoints(farPoints);
  const farMat = new THREE.LineBasicMaterial({ color: FRUSTUM_COLOR, transparent: true, opacity: 0.3 });
  const farLine = new THREE.Line(farGeo, farMat);
  group.add(farLine);

  // Edge lines connecting near to far
  const edges = [
    [nearPoints[0], farPoints[0]],
    [nearPoints[1], farPoints[1]],
    [nearPoints[2], farPoints[2]],
    [nearPoints[3], farPoints[3]],
  ];

  edges.forEach(([start, end]) => {
    const edgeGeo = new THREE.BufferGeometry().setFromPoints([start, end]);
    const edgeMat = new THREE.LineBasicMaterial({ color: FRUSTUM_COLOR, transparent: true, opacity: 0.4 });
    const edgeLine = new THREE.Line(edgeGeo, edgeMat);
    group.add(edgeLine);
  });

  // Camera body indicator
  const bodyGeo = new THREE.BoxGeometry(0.03, 0.015, 0.025);
  const bodyMat = new THREE.MeshStandardMaterial({
    color: FRUSTUM_COLOR,
    emissive: FRUSTUM_COLOR,
    emissiveIntensity: 0.3,
  });
  const body = new THREE.Mesh(bodyGeo, bodyMat);
  body.position.set(0, FRUSTUM_NEAR * 0.5, 0);
  group.add(body);

  // Lens circle
  const lensGeo = new THREE.RingGeometry(0.008, 0.012, 16);
  const lensMat = new THREE.MeshStandardMaterial({
    color: 0xffffff,
    emissive: 0xffffff,
    emissiveIntensity: 0.5,
    side: THREE.DoubleSide,
  });
  const lens = new THREE.Mesh(lensGeo, lensMat);
  lens.position.set(0, FRUSTUM_NEAR, 0);
  lens.rotation.x = Math.PI / 2;
  group.add(lens);

  // Position at front of robot
  group.position.set(0, 0.25, 0.13);
  group.rotation.x = Math.PI / 2; // point forward

  return group;
}

export function createSensorLabel(
  text: string,
  position: THREE.Vector3,
  color: number = 0xf4f4f4
): THREE.Sprite {
  const canvas = document.createElement('canvas');
  const ctx = canvas.getContext('2d')!;
  canvas.width = 128;
  canvas.height = 32;

  ctx.fillStyle = 'rgba(22, 22, 22, 0.8)';
  ctx.roundRect(0, 0, 128, 32, 4);
  ctx.fill();

  ctx.fillStyle = `#${color.toString(16).padStart(6, '0')}`;
  ctx.font = '600 18px IBM Plex Mono, monospace';
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  ctx.fillText(text, 64, 16);

  const texture = new THREE.CanvasTexture(canvas);
  texture.minFilter = THREE.LinearFilter;

  const spriteMat = new THREE.SpriteMaterial({
    map: texture,
    transparent: true,
    depthTest: false,
  });
  const sprite = new THREE.Sprite(spriteMat);
  sprite.scale.set(0.12, 0.03, 1);
  sprite.position.copy(position);

  return sprite;
}

export function updateSensorLabel(sprite: THREE.Sprite, distanceMm: number, maxMm: number) {
  const ratio = distanceMm / maxMm;
  let color: number;
  if (ratio < 0.15) color = 0xff2222;
  else if (ratio < 0.3) color = 0xff8800;
  else color = 0x00cc66;

  const text = distanceMm >= 1000 ? `${(distanceMm / 1000).toFixed(1)}m` : `${Math.round(distanceMm)}mm`;

  const canvas = document.createElement('canvas');
  const ctx = canvas.getContext('2d')!;
  canvas.width = 128;
  canvas.height = 32;

  ctx.fillStyle = 'rgba(22, 22, 22, 0.8)';
  ctx.roundRect(0, 0, 128, 32, 4);
  ctx.fill();

  ctx.fillStyle = `#${color.toString(16).padStart(6, '0')}`;
  ctx.font = '600 18px IBM Plex Mono, monospace';
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  ctx.fillText(text, 64, 16);

  const texture = new THREE.CanvasTexture(canvas);
  texture.minFilter = THREE.LinearFilter;
  (sprite.material as THREE.SpriteMaterial).map = texture;
  (sprite.material as THREE.SpriteMaterial).needsUpdate = true;
}
