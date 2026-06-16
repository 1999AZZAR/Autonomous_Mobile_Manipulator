// Waypoint path rendering — 3D ribbon lines with direction arrows

import * as THREE from 'three';
import type { Waypoint3D } from '../types/twin';

const PATH_COLOR = 0x78a9ff;
const PATH_ACTIVE_COLOR = 0x42be65;
const WAYPOINT_SIZE = 0.03;
const ARROW_SIZE = 0.015;

export function createPathLine(waypoints: Waypoint3D[]): THREE.Group {
  const group = new THREE.Group();

  if (waypoints.length < 2) {
    // Single waypoint — just a dot
    if (waypoints.length === 1) {
      const dot = createWaypointDot(waypoints[0], PATH_COLOR);
      group.add(dot);
    }
    return group;
  }

  // Sort by order
  const sorted = [...waypoints].sort((a, b) => a.order - b.order);

  const points = sorted.map((wp) => new THREE.Vector3(wp.x, wp.y, 0.05));
  const lineGeo = new THREE.BufferGeometry().setFromPoints(points);
  const lineMat = new THREE.LineBasicMaterial({ color: PATH_COLOR, linewidth: 2 });
  const line = new THREE.Line(lineGeo, lineMat);
  group.add(line);

  // Create waypoint dots and direction arrows
  sorted.forEach((wp, i) => {
    const dot = createWaypointDot(wp, PATH_COLOR);
    group.add(dot);

    // Direction arrow between waypoints
    if (i < sorted.length - 1) {
      const next = sorted[i + 1];
      const arrow = createDirectionArrow(wp, next, PATH_COLOR);
      if (arrow) group.add(arrow);
    }
  });

  return group;
}

export function highlightWaypoint(group: THREE.Group, order: number, color: number = PATH_ACTIVE_COLOR) {
  group.children.forEach((child) => {
    if (child instanceof THREE.Mesh && child.userData.waypointOrder === order) {
      const mat = child.material as THREE.MeshStandardMaterial;
      mat.emissiveIntensity = 1.0;
      mat.color.setHex(color);
      mat.emissive.setHex(color);
    }
  });
}

export function resetPathHighlight(group: THREE.Group, color: number = PATH_COLOR) {
  group.children.forEach((child) => {
    if (child instanceof THREE.Mesh && child.userData.waypointOrder !== undefined) {
      const mat = child.material as THREE.MeshStandardMaterial;
      mat.emissiveIntensity = 0.3;
      mat.color.setHex(color);
      mat.emissive.setHex(color);
    }
  });
}

function createWaypointDot(wp: Waypoint3D, color: number): THREE.Mesh {
  const geo = new THREE.SphereGeometry(WAYPOINT_SIZE, 12, 12);
  const mat = new THREE.MeshStandardMaterial({
    color,
    emissive: color,
    emissiveIntensity: 0.3,
    transparent: true,
    opacity: 0.9,
  });
  const mesh = new THREE.Mesh(geo, mat);
  mesh.position.set(wp.x, wp.y, 0.05);
  mesh.userData.waypointOrder = wp.order;
  mesh.castShadow = true;
  return mesh;
}

function createDirectionArrow(from: Waypoint3D, to: Waypoint3D, color: number): THREE.Group | null {
  const fx = from.x;
  const fy = from.y;
  const tx = to.x;
  const ty = to.y;

  const dx = tx - fx;
  const dy = ty - fy;
  const dist = Math.sqrt(dx * dx + dy * dy);

  if (dist < 0.01) return null;

  const angle = Math.atan2(dy, dx);
  const midX = (fx + tx) / 2;
  const midY = (fy + ty) / 2;

  const group = new THREE.Group();

  // Arrow cone
  const coneGeo = new THREE.ConeGeometry(ARROW_SIZE, ARROW_SIZE * 2, 6);
  const coneMat = new THREE.MeshStandardMaterial({ color, emissive: color, emissiveIntensity: 0.2 });
  const cone = new THREE.Mesh(coneGeo, coneMat);
  cone.position.set(midX, midY, 0.05);
  cone.rotation.z = angle - Math.PI / 2;
  cone.castShadow = true;
  group.add(cone);

  return group;
}

export function createReplayMarker(position: { x: number; y: number }, heading: number): THREE.Group {
  const group = new THREE.Group();

  // Glowing ring
  const ringGeo = new THREE.RingGeometry(0.04, 0.055, 24);
  const ringMat = new THREE.MeshStandardMaterial({
    color: 0x42be65,
    emissive: 0x42be65,
    emissiveIntensity: 0.8,
    transparent: true,
    opacity: 0.7,
    side: THREE.DoubleSide,
  });
  const ring = new THREE.Mesh(ringGeo, ringMat);
  ring.rotation.x = -Math.PI / 2;
  ring.position.z = 0.002;
  group.add(ring);

  // Center dot
  const dotGeo = new THREE.CircleGeometry(0.02, 16);
  const dotMat = new THREE.MeshStandardMaterial({
    color: 0x42be65,
    emissive: 0x42be65,
    emissiveIntensity: 1.0,
  });
  const dot = new THREE.Mesh(dotGeo, dotMat);
  dot.rotation.x = -Math.PI / 2;
  dot.position.z = 0.003;
  group.add(dot);

  group.position.set(position.x, position.y, 0);
  group.rotation.z = (heading * Math.PI) / 180;

  return group;
}
