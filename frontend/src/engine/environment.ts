// Floor, walls, obstacles — environment visualization

import * as THREE from 'three';
import type { Obstacle } from '../types/twin';

export function createEnvironment(scene: THREE.Scene) {
  // Scale reference — 1m cubes at origin corners
  const scaleMat = new THREE.MeshStandardMaterial({ color: 0x555577, roughness: 0.5 });

  for (let i = 0; i < 4; i++) {
    const geo = new THREE.BoxGeometry(0.05, 0.05, 0.05);
    const marker = new THREE.Mesh(geo, scaleMat);
    const x = (i % 2 === 0 ? -1 : 1) * 4.5;
    const y = (i < 2 ? -1 : 1) * 4.5;
    marker.position.set(x, y, 0.025);
    marker.castShadow = true;
    scene.add(marker);
  }

  // Axis markers at center
  const axisLen = 0.3;

  const xGeo = new THREE.CylinderGeometry(0.005, 0.005, axisLen, 6);
  const xMat = new THREE.MeshStandardMaterial({ color: 0xff4444 });
  const xAxis = new THREE.Mesh(xGeo, xMat);
  xAxis.position.set(axisLen / 2, 0, 0.005);
  xAxis.rotation.z = Math.PI / 2;
  scene.add(xAxis);

  const yGeo = new THREE.CylinderGeometry(0.005, 0.005, axisLen, 6);
  const yMat = new THREE.MeshStandardMaterial({ color: 0x44ff44 });
  const yAxis = new THREE.Mesh(yGeo, yMat);
  yAxis.position.set(0, axisLen / 2, 0.005);
  yAxis.rotation.x = Math.PI / 2;
  scene.add(yAxis);

  // Boundary walls (transparent)
  const wallMat = new THREE.MeshStandardMaterial({
    color: 0x444466,
    transparent: true,
    opacity: 0.15,
    side: THREE.DoubleSide,
  });

  const wallSize = 5;
  const wallHeight = 0.3;

  const walls = [
    { x: 0, y: wallSize, rx: 0, ry: 0 },
    { x: 0, y: -wallSize, rx: 0, ry: 0 },
    { x: wallSize, y: 0, rx: 0, ry: Math.PI / 2 },
    { x: -wallSize, y: 0, rx: 0, ry: Math.PI / 2 },
  ];

  walls.forEach((w) => {
    const geo = new THREE.PlaneGeometry(wallSize * 2, wallHeight);
    const wall = new THREE.Mesh(geo, wallMat);
    wall.position.set(w.x, w.y, wallHeight / 2);
    wall.rotation.set(w.rx, w.ry, 0);
    scene.add(wall);
  });
}

export function createObstacleMesh(obstacle: Obstacle): THREE.Mesh {
  let geo: THREE.BufferGeometry;

  if (obstacle.type === 'cylinder') {
    geo = new THREE.CylinderGeometry(
      obstacle.width / 2,
      obstacle.width / 2,
      obstacle.height,
      16
    );
  } else {
    geo = new THREE.BoxGeometry(obstacle.width, obstacle.height, obstacle.depth);
  }

  const mat = new THREE.MeshStandardMaterial({
    color: obstacle.color ?? 0x8b4513,
    roughness: 0.7,
    transparent: true,
    opacity: 0.8,
  });

  const mesh = new THREE.Mesh(geo, mat);
  mesh.position.set(obstacle.x, obstacle.y, obstacle.depth / 2 || obstacle.height / 2);
  mesh.castShadow = true;
  mesh.receiveShadow = true;
  return mesh;
}

export function updateObstacles(
  scene: THREE.Scene,
  oldMeshes: THREE.Mesh[],
  newObstacles: Obstacle[]
): THREE.Mesh[] {
  // Remove old
  oldMeshes.forEach((m) => {
    scene.remove(m);
    m.geometry.dispose();
    (m.material as THREE.Material).dispose();
  });

  // Add new
  const newMeshes = newObstacles.map((obs) => {
    const mesh = createObstacleMesh(obs);
    scene.add(mesh);
    return mesh;
  });

  return newMeshes;
}
