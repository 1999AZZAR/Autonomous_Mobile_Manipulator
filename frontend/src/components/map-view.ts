// Digital map — canvas-based robot position + waypoint + sensor visualization

import { fetchRobotPosition, fetchSensors, fetchPaths, fetchPath, fetchWaypointStatus } from '../api';
import type { RobotPosition, SensorReadings, MapWaypoint, SavedPath } from '../types';

let canvas: HTMLCanvasElement | null = null;
let ctx: CanvasRenderingContext2D | null = null;
let pollTimer: ReturnType<typeof setInterval> | null = null;
let robotPos: RobotPosition = { x: 0, y: 0, z: 0, roll: 0, pitch: 0, yaw: 0 };
let sensors: Partial<SensorReadings> = {};
let pathWaypoints: MapWaypoint[] = [];
let selectedPathId: number | null = null;
let allPaths: SavedPath[] = [];
let wpStatus = { recording: false, replaying: false, replay_index: 0 };

const SCALE = 60; // pixels per meter
const CENTER_X = 300;
const CENTER_Y = 250;
const ROBOT_SIZE = 16;

function el(tag: string, cls?: string, text?: string): HTMLElement {
  const e = document.createElement(tag);
  if (cls) e.className = cls;
  if (text) e.textContent = text;
  return e;
}

function worldToCanvas(x: number, y: number): [number, number] {
  return [CENTER_X + x * SCALE, CENTER_Y - y * SCALE];
}

function drawGrid() {
  if (!ctx || !canvas) return;
  ctx.clearRect(0, 0, canvas.width, canvas.height);

  // Grid
  ctx.strokeStyle = '#2a2a2a';
  ctx.lineWidth = 0.5;
  for (let gx = -10; gx <= 10; gx++) {
    const [sx] = worldToCanvas(gx, 0);
    ctx.beginPath();
    ctx.moveTo(sx, 0);
    ctx.lineTo(sx, canvas.height);
    ctx.stroke();
  }
  for (let gy = -10; gy <= 10; gy++) {
    const [, sy] = worldToCanvas(0, gy);
    ctx.beginPath();
    ctx.moveTo(0, sy);
    ctx.lineTo(canvas.width, sy);
    ctx.stroke();
  }

  // Origin cross
  const [ox, oy] = worldToCanvas(0, 0);
  ctx.strokeStyle = '#555';
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(ox - 8, oy);
  ctx.lineTo(ox + 8, oy);
  ctx.moveTo(ox, oy - 8);
  ctx.lineTo(ox, oy + 8);
  ctx.stroke();

  // Axis labels
  ctx.fillStyle = '#666';
  ctx.font = '10px IBM Plex Mono';
  ctx.fillText('X', ox + 12, oy + 4);
  ctx.fillText('Y', ox - 4, oy - 12);

  // Scale marker
  ctx.fillStyle = '#555';
  ctx.fillText('1m', ox + SCALE - 5, oy + 16);
  ctx.strokeStyle = '#555';
  ctx.beginPath();
  ctx.moveTo(ox, oy + 12);
  ctx.lineTo(ox + SCALE, oy + 12);
  ctx.stroke();
}

function drawWaypoints() {
  if (!ctx || pathWaypoints.length === 0) return;

  // Draw path line
  ctx.strokeStyle = 'rgba(69, 137, 255, 0.5)';
  ctx.lineWidth = 2;
  ctx.setLineDash([4, 4]);
  ctx.beginPath();
  pathWaypoints.forEach((wp, i) => {
    const [cx, cy] = worldToCanvas(wp.x, wp.y);
    if (i === 0) ctx!.moveTo(cx, cy);
    else ctx!.lineTo(cx, cy);
  });
  ctx.stroke();
  ctx.setLineDash([]);

  // Draw waypoint dots
  pathWaypoints.forEach((wp, i) => {
    const [cx, cy] = worldToCanvas(wp.x, wp.y);
    const isCurrent = wpStatus.replaying && i === wpStatus.replay_index;

    ctx!.beginPath();
    ctx!.arc(cx, cy, isCurrent ? 6 : 4, 0, Math.PI * 2);
    ctx!.fillStyle = isCurrent ? '#f1c21b' : 'rgba(69, 137, 255, 0.8)';
    ctx!.fill();

    // Heading arrow
    const rad = (wp.heading * Math.PI) / 180;
    const arrowLen = 12;
    ctx!.strokeStyle = isCurrent ? '#f1c21b' : 'rgba(69, 137, 255, 0.6)';
    ctx!.lineWidth = 1.5;
    ctx!.beginPath();
    ctx!.moveTo(cx, cy);
    ctx!.lineTo(cx + Math.cos(rad) * arrowLen, cy - Math.sin(rad) * arrowLen);
    ctx!.stroke();

    // Label
    ctx!.fillStyle = '#888';
    ctx!.font = '9px IBM Plex Mono';
    ctx!.fillText(`${i}`, cx + 8, cy - 4);
  });
}

function drawSensorArcs() {
  if (!ctx) return;

  const [rx, ry] = worldToCanvas(robotPos.x, robotPos.y);
  const yawRad = (robotPos.yaw * Math.PI) / 180;

  // Sensor directions relative to robot heading
  const sensorDefs: Array<{ key: string; angle: number; range: number; color: string }> = [
    { key: 'laser_left_front', angle: yawRad + Math.PI / 4, range: 50, color: 'rgba(66, 190, 101, 0.3)' },
    { key: 'laser_right_front', angle: yawRad - Math.PI / 4, range: 50, color: 'rgba(66, 190, 101, 0.3)' },
    { key: 'laser_left_back', angle: yawRad + (3 * Math.PI) / 4, range: 50, color: 'rgba(250, 77, 86, 0.3)' },
    { key: 'laser_right_back', angle: yawRad - (3 * Math.PI) / 4, range: 50, color: 'rgba(250, 77, 86, 0.3)' },
    { key: 'ultra_front_left', angle: yawRad + Math.PI / 8, range: 30, color: 'rgba(69, 137, 255, 0.25)' },
    { key: 'ultra_front_right', angle: yawRad - Math.PI / 8, range: 30, color: 'rgba(69, 137, 255, 0.25)' },
  ];

  sensorDefs.forEach(def => {
    const val = sensors[def.key as keyof SensorReadings];
    if (val == null || val === 0) return;

    // Clamp display range
    const displayRange = Math.min(val, def.range) * 0.8;
    if (displayRange < 5) return;

    const arcStart = def.angle - 0.3;
    const arcEnd = def.angle + 0.3;

    ctx!.fillStyle = def.color;
    ctx!.beginPath();
    ctx!.moveTo(rx, ry);
    ctx!.arc(rx, ry, displayRange, -arcEnd, -arcStart);
    ctx!.closePath();
    ctx!.fill();
  });
}

function drawRobot() {
  if (!ctx) return;

  const [rx, ry] = worldToCanvas(robotPos.x, robotPos.y);
  const yawRad = (robotPos.yaw * Math.PI) / 180;

  ctx.save();
  ctx.translate(rx, ry);
  ctx.rotate(-yawRad);

  // Robot body
  ctx.fillStyle = wpStatus.recording ? '#f1c21b' : (wpStatus.replaying ? '#42be65' : '#0f62fe');
  ctx.beginPath();
  ctx.roundRect(-ROBOT_SIZE / 2, -ROBOT_SIZE / 2, ROBOT_SIZE, ROBOT_SIZE, 3);
  ctx.fill();

  // Direction arrow
  ctx.fillStyle = '#fff';
  ctx.beginPath();
  ctx.moveTo(ROBOT_SIZE / 2, 0);
  ctx.lineTo(ROBOT_SIZE / 2 - 6, -4);
  ctx.lineTo(ROBOT_SIZE / 2 - 6, 4);
  ctx.closePath();
  ctx.fill();

  ctx.restore();

  // Position label
  ctx.fillStyle = '#ccc';
  ctx.font = '10px IBM Plex Mono';
  ctx.fillText(`(${robotPos.x.toFixed(2)}, ${robotPos.y.toFixed(2)})`, rx + ROBOT_SIZE, ry - ROBOT_SIZE);
  ctx.fillText(`H: ${robotPos.yaw.toFixed(1)}°`, rx + ROBOT_SIZE, ry - ROBOT_SIZE + 12);
}

function render() {
  drawGrid();
  drawSensorArcs();
  drawWaypoints();
  drawRobot();
}

async function pollData(): Promise<void> {
  try {
    const [posResp, sens, status] = await Promise.all([
      fetchRobotPosition().catch(() => null),
      fetchSensors().catch(() => ({} as Partial<SensorReadings>)),
      fetchWaypointStatus().catch(() => wpStatus),
    ]);
    if (posResp && posResp.success) {
      robotPos = {
        x: posResp.position.x,
        y: posResp.position.y,
        z: posResp.position.z,
        roll: posResp.orientation.roll,
        pitch: posResp.orientation.pitch,
        yaw: posResp.orientation.yaw,
      };
    }
    sensors = sens;
    wpStatus = { ...wpStatus, ...status };
  } catch {
    // Silent fail
  }
  render();
}

async function loadPathDetail(pathId: number): Promise<void> {
  try {
    const path = await fetchPath(pathId);
    pathWaypoints = path.waypoints.map(w => ({
      x: w.x,
      y: w.y,
      heading: w.heading,
      order: w.order,
    }));
  } catch {
    pathWaypoints = [];
  }
  render();
}

export function renderMap(container: HTMLElement): void {
  container.innerHTML = '';

  // Header
  const header = el('div', 'map-header');
  header.appendChild(el('h2', '', 'Navigation Map'));

  // Status indicators
  const statusRow = el('div', 'map-status-row');

  const posBadge = el('span', 'map-badge', 'Pos: --');
  posBadge.id = 'map-pos-badge';
  statusRow.appendChild(posBadge);

  const recBadge = el('span', 'map-badge', 'IDLE');
  recBadge.id = 'map-rec-badge';
  statusRow.appendChild(recBadge);

  header.appendChild(statusRow);
  container.appendChild(header);

  // Canvas wrapper
  const canvasWrap = el('div', 'map-canvas-wrap');
  canvas = document.createElement('canvas');
  canvas.width = 600;
  canvas.height = 500;
  canvas.id = 'map-canvas';
  ctx = canvas.getContext('2d');
  canvasWrap.appendChild(canvas);
  container.appendChild(canvasWrap);

  // Path selector
  const pathPanel = el('div', 'map-path-panel');

  const pathSelect = document.createElement('select');
  pathSelect.id = 'map-path-select';
  pathSelect.className = 'input';
  const defaultOpt = document.createElement('option');
  defaultOpt.value = '';
  defaultOpt.textContent = 'Select path to display';
  pathSelect.appendChild(defaultOpt);
  pathSelect.onchange = async () => {
    const val = pathSelect.value;
    if (val) {
      selectedPathId = parseInt(val);
      await loadPathDetail(selectedPathId);
    } else {
      selectedPathId = null;
      pathWaypoints = [];
      render();
    }
  };
  pathPanel.appendChild(pathSelect);
  container.appendChild(pathPanel);

  // Legend
  const legend = el('div', 'map-legend');
  legend.innerHTML = `
    <span class="map-legend-item"><span class="map-legend-dot" style="background:#0f62fe"></span>Robot</span>
    <span class="map-legend-item"><span class="map-legend-dot" style="background:#4589ff"></span>Waypoints</span>
    <span class="map-legend-item"><span class="map-legend-dot" style="background:#42be65"></span>Laser</span>
    <span class="map-legend-item"><span class="map-legend-dot" style="background:#4589ff;opacity:0.5"></span>Ultrasonic</span>
    <span class="map-legend-item"><span class="map-legend-dot" style="background:#f1c21b"></span>Active/Replay</span>
  `;
  container.appendChild(legend);

  // Load paths
  refreshPathList();

  // Start polling
  if (pollTimer) clearInterval(pollTimer);
  pollData();
  pollTimer = setInterval(pollData, 500);
}

async function refreshPathList(): Promise<void> {
  const select = document.getElementById('map-path-select') as HTMLSelectElement | null;
  if (!select) return;

  try {
    const { paths } = await fetchPaths();
    allPaths = paths;
    select.innerHTML = '';
    const defaultOpt = document.createElement('option');
    defaultOpt.value = '';
    defaultOpt.textContent = 'Select path to display';
    select.appendChild(defaultOpt);
    paths.forEach(p => {
      const opt = document.createElement('option');
      opt.value = String(p.id);
      opt.textContent = `${p.name} (${p.waypoint_count} wp)`;
      select.appendChild(opt);
    });
  } catch {
    // Silent
  }
}

export function destroyMap(): void {
  if (pollTimer) {
    clearInterval(pollTimer);
    pollTimer = null;
  }
}
