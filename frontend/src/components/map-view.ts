// Digital map — canvas-based robot position + waypoint + sensor + drawing + training

import { fetchRobotPosition, fetchSensors, fetchPaths, fetchPath, fetchWaypointStatus, sendNavigationGoal } from '../api';
import type { RobotPosition, SensorReadings, MapWaypoint, SavedPath } from '../types';
import { WorldState } from '../state/world-state';
import { syncMapObstaclesToTwin } from './digital-twin';

let canvas: HTMLCanvasElement | null = null;
let ctx: CanvasRenderingContext2D | null = null;
let pollTimer: ReturnType<typeof setInterval> | null = null;
let robotPos: RobotPosition = { x: 0, y: 0, z: 0, roll: 0, pitch: 0, yaw: 0 };
let sensors: Partial<SensorReadings> = {};
let pathWaypoints: MapWaypoint[] = [];
let selectedPathId: number | null = null;
let allPaths: SavedPath[] = [];
let wpStatus = { recording: false, replaying: false, replay_index: 0 };

const SCALE = 60;
const CENTER_X = 300;
const CENTER_Y = 250;
const ROBOT_SIZE = 16;
const GRID_W = 60;
const GRID_H = 50;

// --- Map mode ---
type MapMode = 'navigate' | 'draw-wall' | 'draw-obstacle' | 'erase' | 'train';
let mapMode: MapMode = 'navigate';

// --- Drawing state ---
interface DrawnElement {
  type: 'wall' | 'obstacle';
  x1: number; y1: number;
  x2: number; y2: number;
  width: number;
}
let drawnElements: DrawnElement[] = [];
let drawStart: { x: number; y: number } | null = null;

// --- Training / occupancy grid ---
let occupancyGrid: Float32Array = new Float32Array(GRID_W * GRID_H);
let trainingActive = false;
let gridOriginX = 0;
let gridOriginY = 0;

function el(tag: string, cls?: string, text?: string): HTMLElement {
  const e = document.createElement(tag);
  if (cls) e.className = cls;
  if (text) e.textContent = text;
  return e;
}

function worldToCanvas(x: number, y: number): [number, number] {
  return [CENTER_X + x * SCALE, CENTER_Y - y * SCALE];
}

function canvasToWorld(cx: number, cy: number): [number, number] {
  return [(cx - CENTER_X) / SCALE, (CENTER_Y - cy) / SCALE];
}

function drawGrid() {
  if (!ctx || !canvas) return;
  ctx.clearRect(0, 0, canvas.width, canvas.height);

  ctx.strokeStyle = '#2a2a2a';
  ctx.lineWidth = 0.5;
  for (let gx = -10; gx <= 10; gx++) {
    const [sx] = worldToCanvas(gx, 0);
    ctx.beginPath(); ctx.moveTo(sx, 0); ctx.lineTo(sx, canvas.height); ctx.stroke();
  }
  for (let gy = -10; gy <= 10; gy++) {
    const [, sy] = worldToCanvas(0, gy);
    ctx.beginPath(); ctx.moveTo(0, sy); ctx.lineTo(canvas.width, sy); ctx.stroke();
  }

  const [ox, oy] = worldToCanvas(0, 0);
  ctx.strokeStyle = '#555'; ctx.lineWidth = 1;
  ctx.beginPath(); ctx.moveTo(ox - 8, oy); ctx.lineTo(ox + 8, oy); ctx.moveTo(ox, oy - 8); ctx.lineTo(ox, oy + 8); ctx.stroke();
  ctx.fillStyle = '#666'; ctx.font = '10px IBM Plex Mono';
  ctx.fillText('X', ox + 12, oy + 4); ctx.fillText('Y', ox - 4, oy - 12);
  ctx.fillStyle = '#555'; ctx.fillText('1m', ox + SCALE - 5, oy + 16);
  ctx.strokeStyle = '#555'; ctx.beginPath(); ctx.moveTo(ox, oy + 12); ctx.lineTo(ox + SCALE, oy + 12); ctx.stroke();
}

function drawOccupancyGrid() {
  if (!ctx || !canvas) return;
  const cellW = SCALE;
  const cellH = SCALE;

  for (let gy = 0; gy < GRID_H; gy++) {
    for (let gx = 0; gx < GRID_W; gx++) {
      const prob = occupancyGrid[gy * GRID_W + gx];
      if (prob <= 0.1) continue;

      const wx = gridOriginX + gx;
      const wy = gridOriginY + gy;
      const [cx, cy] = worldToCanvas(wx / GRID_W * 10 - 5, wy / GRID_H * 10 - 5);
      const cw = cellW / GRID_W * 10;
      const ch = cellH / GRID_H * 10;

      const intensity = Math.min(prob, 1);
      ctx.fillStyle = `rgba(250, 77, 86, ${0.15 + intensity * 0.6})`;
      ctx.fillRect(cx - cw / 2, cy - ch / 2, cw, ch);
    }
  }
}

function drawnElementsOnCanvas() {
  if (!ctx) return;
  const c = ctx;
  drawnElements.forEach((el) => {
    const [x1, y1] = worldToCanvas(el.x1, el.y1);
    const [x2, y2] = worldToCanvas(el.x2, el.y2);

    if (el.type === 'wall') {
      c.strokeStyle = 'rgba(255, 160, 0, 0.7)';
      c.lineWidth = el.width * SCALE;
      c.lineCap = 'round';
      c.beginPath(); c.moveTo(x1, y1); c.lineTo(x2, y2); c.stroke();
    } else {
      const minX = Math.min(x1, x2), minY = Math.min(y1, y2);
      const w = Math.abs(x2 - x1), h = Math.abs(y2 - y1);
      c.fillStyle = 'rgba(139, 69, 19, 0.5)';
      c.fillRect(minX, minY, w, h);
      c.strokeStyle = 'rgba(139, 69, 19, 0.8)';
      c.lineWidth = 1;
      c.strokeRect(minX, minY, w, h);
    }
  });

  // Draw in-progress element
  if (drawStart && mapMode.startsWith('draw')) {
    const mouse = lastMouse;
    if (mouse) {
      const [x1, y1] = worldToCanvas(drawStart.x, drawStart.y);
      c.strokeStyle = mapMode === 'draw-wall' ? 'rgba(255, 160, 0, 0.4)' : 'rgba(139, 69, 19, 0.4)';
      c.lineWidth = mapMode === 'draw-wall' ? 4 : 1;
      c.setLineDash([4, 4]);
      c.beginPath(); c.moveTo(x1, y1); c.lineTo(mouse.cx, mouse.cy); c.stroke();
      c.setLineDash([]);
    }
  }
}

function drawWaypoints() {
  if (!ctx || pathWaypoints.length === 0) return;

  ctx.strokeStyle = 'rgba(69, 137, 255, 0.5)';
  ctx.lineWidth = 2;
  ctx.setLineDash([4, 4]);
  ctx.beginPath();
  pathWaypoints.forEach((wp, i) => {
    const [cx, cy] = worldToCanvas(wp.x, wp.y);
    if (i === 0) ctx!.moveTo(cx, cy); else ctx!.lineTo(cx, cy);
  });
  ctx.stroke();
  ctx.setLineDash([]);

  pathWaypoints.forEach((wp, i) => {
    const [cx, cy] = worldToCanvas(wp.x, wp.y);
    const isCurrent = wpStatus.replaying && i === wpStatus.replay_index;
    ctx!.beginPath(); ctx!.arc(cx, cy, isCurrent ? 6 : 4, 0, Math.PI * 2);
    ctx!.fillStyle = isCurrent ? '#f1c21b' : 'rgba(69, 137, 255, 0.8)'; ctx!.fill();
    const rad = (wp.heading * Math.PI) / 180;
    ctx!.strokeStyle = isCurrent ? '#f1c21b' : 'rgba(69, 137, 255, 0.6)';
    ctx!.lineWidth = 1.5; ctx!.beginPath(); ctx!.moveTo(cx, cy);
    ctx!.lineTo(cx + Math.cos(rad) * 12, cy - Math.sin(rad) * 12); ctx!.stroke();
    ctx!.fillStyle = '#888'; ctx!.font = '9px IBM Plex Mono'; ctx!.fillText(`${i}`, cx + 8, cy - 4);
  });
}

function drawSensorArcs() {
  if (!ctx) return;
  const [rx, ry] = worldToCanvas(robotPos.x, robotPos.y);
  const yawRad = (robotPos.yaw * Math.PI) / 180;

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
    const displayRange = Math.min(val, def.range) * 0.8;
    if (displayRange < 5) return;
    const arcStart = def.angle - 0.3;
    const arcEnd = def.angle + 0.3;
    ctx!.fillStyle = def.color; ctx!.beginPath(); ctx!.moveTo(rx, ry);
    ctx!.arc(rx, ry, displayRange, -arcEnd, -arcStart); ctx!.closePath(); ctx!.fill();
  });
}

function drawRobot() {
  if (!ctx) return;
  const [rx, ry] = worldToCanvas(robotPos.x, robotPos.y);
  const yawRad = (robotPos.yaw * Math.PI) / 180;
  ctx.save(); ctx.translate(rx, ry); ctx.rotate(-yawRad);
  ctx.fillStyle = wpStatus.recording ? '#f1c21b' : (wpStatus.replaying ? '#42be65' : '#0f62fe');
  ctx.beginPath(); ctx.roundRect(-ROBOT_SIZE / 2, -ROBOT_SIZE / 2, ROBOT_SIZE, ROBOT_SIZE, 3); ctx.fill();
  ctx.fillStyle = '#fff'; ctx.beginPath(); ctx.moveTo(ROBOT_SIZE / 2, 0);
  ctx.lineTo(ROBOT_SIZE / 2 - 6, -4); ctx.lineTo(ROBOT_SIZE / 2 - 6, 4); ctx.closePath(); ctx.fill();
  ctx.restore();
  ctx.fillStyle = '#ccc'; ctx.font = '10px IBM Plex Mono';
  ctx.fillText(`(${robotPos.x.toFixed(2)}, ${robotPos.y.toFixed(2)})`, rx + ROBOT_SIZE, ry - ROBOT_SIZE);
  ctx.fillText(`H: ${robotPos.yaw.toFixed(1)}°`, rx + ROBOT_SIZE, ry - ROBOT_SIZE + 12);
}

function render() {
  drawGrid();
  drawOccupancyGrid();
  drawnElementsOnCanvas();
  drawSensorArcs();
  drawWaypoints();
  drawRobot();
}

// --- Training: update occupancy grid from sensor data ---
function trainFromSensors() {
  if (!trainingActive) return;

  const hx = robotPos.x;
  const hy = robotPos.y;
  const yawRad = (robotPos.yaw * Math.PI) / 180;

  const sensorDefs = [
    { key: 'laser_left_front', angle: yawRad + Math.PI / 4, dist: sensors.laser_left_front },
    { key: 'laser_right_front', angle: yawRad - Math.PI / 4, dist: sensors.laser_right_front },
    { key: 'laser_left_back', angle: yawRad + (3 * Math.PI) / 4, dist: sensors.laser_left_back },
    { key: 'laser_right_back', angle: yawRad - (3 * Math.PI) / 4, dist: sensors.laser_right_back },
    { key: 'ultra_front_left', angle: yawRad + Math.PI / 8, dist: sensors.ultra_front_left },
    { key: 'ultra_front_right', angle: yawRad - Math.PI / 8, dist: sensors.ultra_front_right },
  ];

  sensorDefs.forEach((s) => {
    if (s.dist == null || s.dist <= 0) return;
    const hitX = hx + Math.cos(s.angle) * s.dist * 0.001;
    const hitY = hy + Math.sin(s.angle) * s.dist * 0.001;

    // Mark hit point as occupied
    markOccupied(hitX, hitY);

    // Mark cells along ray as free
    const steps = Math.ceil(s.dist * 0.001 * SCALE / 2);
    for (let i = 0; i < steps; i++) {
      const t = i / steps;
      const fx = hx + (hitX - hx) * t;
      const fy = hy + (hitY - hy) * t;
      markFree(fx, fy);
    }
  });

  // Mark cells around robot as free
  for (let dx = -1; dx <= 1; dx++) {
    for (let dy = -1; dy <= 1; dy++) {
      markFree(hx + dx * 0.1, hy + dy * 0.1);
    }
  }
}

function worldToGrid(wx: number, wy: number): [number, number] | null {
  const gx = Math.floor((wx + 5) / 10 * GRID_W);
  const gy = Math.floor((wy + 5) / 10 * GRID_H);
  if (gx < 0 || gx >= GRID_W || gy < 0 || gy >= GRID_H) return null;
  return [gx, gy];
}

function markOccupied(wx: number, wy: number) {
  const cell = worldToGrid(wx, wy);
  if (!cell) return;
  const [gx, gy] = cell;
  const idx = gy * GRID_W + gx;
  occupancyGrid[idx] = Math.min(occupancyGrid[idx] + 0.15, 1);
  WorldState.setOccupancyCell(gx, gy, occupancyGrid[idx] > 0.5);
}

function markFree(wx: number, wy: number) {
  const cell = worldToGrid(wx, wy);
  if (!cell) return;
  const [gx, gy] = cell;
  const idx = gy * GRID_W + gx;
  occupancyGrid[idx] = Math.max(occupancyGrid[idx] - 0.05, 0);
  WorldState.setOccupancyCell(gx, gy, occupancyGrid[idx] > 0.5);
}

function syncOccupancyGridToWorld() {
  for (let gy = 0; gy < GRID_H; gy++) {
    for (let gx = 0; gx < GRID_W; gx++) {
      const idx = gy * GRID_W + gx;
      if (idx < occupancyGrid.length) {
        WorldState.setOccupancyCell(gx, gy, occupancyGrid[idx] > 0.5);
      }
    }
  }
}

export function getOccupancyGrid() { return occupancyGrid; }
export function getGridDims() { return { w: GRID_W, h: GRID_H }; }
export function isTrainingActive() { return trainingActive; }

// --- Drawing interaction ---
let lastMouse: { cx: number; cy: number } | null = null;

function setupDrawing(canvasEl: HTMLCanvasElement) {
  canvasEl.addEventListener('mousedown', (e) => {
    if (e.button !== 0) return;
    const rect = canvasEl.getBoundingClientRect();
    const cx = e.clientX - rect.left;
    const cy = e.clientY - rect.top;
    const [wx, wy] = canvasToWorld(cx, cy);

    if (mapMode === 'navigate') {
      sendNavigationGoal(wx, wy).catch(() => {});
      return;
    }

    if (!mapMode.startsWith('draw') && mapMode !== 'erase') return;

    if (mapMode === 'erase') {
      let minDist = 0.3;
      let minIdx = -1;
      drawnElements.forEach((el, i) => {
        const dx = wx - (el.x1 + el.x2) / 2;
        const dy = wy - (el.y1 + el.y2) / 2;
        const d = Math.sqrt(dx * dx + dy * dy);
        if (d < minDist) { minDist = d; minIdx = i; }
      });
      if (minIdx >= 0) drawnElements.splice(minIdx, 1);
      render();
      return;
    }

    drawStart = { x: wx, y: wy };
  });

  canvasEl.addEventListener('mousemove', (e) => {
    if (!drawStart) return;
    const rect = canvasEl.getBoundingClientRect();
    lastMouse = { cx: e.clientX - rect.left, cy: e.clientY - rect.top };
    render();
  });

  canvasEl.addEventListener('mouseup', (e) => {
    if (!drawStart || !mapMode.startsWith('draw')) return;
    const rect = canvasEl.getBoundingClientRect();
    const cx = e.clientX - rect.left;
    const cy = e.clientY - rect.top;
    const [wx, wy] = canvasToWorld(cx, cy);

    const dx = wx - drawStart.x;
    const dy = wy - drawStart.y;
    if (Math.sqrt(dx * dx + dy * dy) > 0.05) {
      drawnElements.push({
        type: mapMode === 'draw-wall' ? 'wall' : 'obstacle',
        x1: drawStart.x, y1: drawStart.y,
        x2: wx, y2: wy,
        width: mapMode === 'draw-wall' ? 0.15 : 0,
      });
    }
    drawStart = null;
    lastMouse = null;
    render();
    syncDrawnElementsToWorld();
  });
}

function syncDrawnElementsToWorld() {
  const obstacles = drawnElements
    .filter((e) => e.type === 'obstacle' || e.type === 'wall')
    .map((e) => ({
      x: ((e.x1 + e.x2) / 2) * 1000,
      y: ((e.y1 + e.y2) / 2) * 1000,
      width: Math.abs(e.x1 - e.x2) * 1000,
      height: 300,
      depth: Math.abs(e.y1 - e.y2) * 1000,
    }));
  WorldState.removeAllObstacles();
  obstacles.forEach((o) => WorldState.addObstacle(o));
  WorldState.update('map-draw', {
    obstacles: WorldState.get().obstacles,
  });
  syncMapObstaclesToTwin(obstacles);
}

// --- Polling ---
async function pollData(): Promise<void> {
  try {
    const [posResp, sens, status] = await Promise.all([
      fetchRobotPosition().catch(() => null),
      fetchSensors().catch(() => ({} as Partial<SensorReadings>)),
      fetchWaypointStatus().catch(() => wpStatus),
    ]);
    if (posResp && posResp.success) {
      robotPos = {
        x: posResp.position.x, y: posResp.position.y, z: posResp.position.z,
        roll: posResp.orientation.roll, pitch: posResp.orientation.pitch, yaw: posResp.orientation.yaw,
      };
      WorldState.update('real', {
        robotPosition: { x: posResp.position.x, y: posResp.position.y, heading: posResp.orientation.yaw },
        robotPosition3D: posResp.position,
      });
    }
    sensors = sens;
    WorldState.update('real', { sensors });
    wpStatus = { ...wpStatus, ...status };
    if (trainingActive) {
      trainFromSensors();
      syncOccupancyGridToWorld();
    }
  } catch { /* silent */ }
  render();
}

async function loadPathDetail(pathId: number): Promise<void> {
  try {
    const path = await fetchPath(pathId);
    pathWaypoints = path.waypoints.map(w => ({ x: w.x, y: w.y, heading: w.heading, order: w.order }));
  } catch { pathWaypoints = []; }
  render();
}

async function refreshPathList(): Promise<void> {
  const select = document.getElementById('map-path-select') as HTMLSelectElement | null;
  if (!select) return;
  try {
    const { paths } = await fetchPaths();
    allPaths = paths;
    select.innerHTML = '';
    const d = document.createElement('option'); d.value = ''; d.textContent = 'Select path to display';
    select.appendChild(d);
    paths.forEach(p => {
      const o = document.createElement('option'); o.value = String(p.id);
      o.textContent = `${p.name} (${p.waypoint_count} wp)`; select.appendChild(o);
    });
  } catch { /* silent */ }
}

function setMode(newMode: MapMode) {
  mapMode = newMode;
  canvas!.style.cursor = newMode === 'navigate' ? 'default' :
    newMode === 'erase' ? 'crosshair' : 'crosshair';
  updateModeUI();
}

function updateModeUI() {
  document.querySelectorAll('.map-mode-btn').forEach(btn => {
    const b = btn as HTMLButtonElement;
    b.classList.toggle('active', b.dataset.mode === mapMode);
  });
  const trainBtn = document.getElementById('map-train-toggle') as HTMLButtonElement | null;
  if (trainBtn) {
    trainBtn.textContent = trainingActive ? 'Stop Training' : 'Start Training';
    trainBtn.classList.toggle('active', trainingActive);
  }
  const modeLabel = document.getElementById('map-mode-label');
  if (modeLabel) {
    modeLabel.textContent = mapMode.toUpperCase().replace('-', ' ');
  }
}

export function renderMap(container: HTMLElement): void {
  container.innerHTML = '';

  const header = el('div', 'map-header');
  header.appendChild(el('h2', '', 'Navigation Map'));

  const statusRow = el('div', 'map-status-row');
  const posBadge = el('span', 'map-badge', 'Pos: --'); posBadge.id = 'map-pos-badge';
  statusRow.appendChild(posBadge);
  const recBadge = el('span', 'map-badge', 'IDLE'); recBadge.id = 'map-rec-badge';
  statusRow.appendChild(recBadge);
  const modeBadge = el('span', 'map-badge', 'NAVIGATE'); modeBadge.id = 'map-mode-label';
  statusRow.appendChild(modeBadge);
  header.appendChild(statusRow);
  container.appendChild(header);

  // Mode toolbar
  const toolbar = el('div', 'map-toolbar');
  const modes: Array<{ mode: MapMode; label: string }> = [
    { mode: 'navigate', label: 'Navigate' },
    { mode: 'draw-wall', label: '+ Wall' },
    { mode: 'draw-obstacle', label: '+ Box' },
    { mode: 'erase', label: 'Erase' },
  ];
  modes.forEach(m => {
    const btn = document.createElement('button');
    btn.className = 'map-mode-btn' + (m.mode === mapMode ? ' active' : '');
    btn.dataset.mode = m.mode;
    btn.textContent = m.label;
    btn.addEventListener('click', () => setMode(m.mode));
    toolbar.appendChild(btn);
  });
  const trainBtn = document.createElement('button');
  trainBtn.id = 'map-train-toggle';
  trainBtn.className = 'map-mode-btn';
  trainBtn.textContent = 'Start Training';
  trainBtn.addEventListener('click', () => {
    trainingActive = !trainingActive;
    if (trainingActive) occupancyGrid.fill(0);
    updateModeUI();
  });
  toolbar.appendChild(trainBtn);
  container.appendChild(toolbar);

  // Canvas
  const canvasWrap = el('div', 'map-canvas-wrap');
  canvas = document.createElement('canvas');
  canvas.width = 600; canvas.height = 500; canvas.id = 'map-canvas';
  ctx = canvas.getContext('2d');
  canvasWrap.appendChild(canvas);
  container.appendChild(canvasWrap);
  setupDrawing(canvas);

  // Path selector
  const pathPanel = el('div', 'map-path-panel');
  const pathSelect = document.createElement('select');
  pathSelect.id = 'map-path-select'; pathSelect.className = 'input';
  const d = document.createElement('option'); d.value = ''; d.textContent = 'Select path to display';
  pathSelect.appendChild(d);
  pathSelect.onchange = async () => {
    const val = pathSelect.value;
    if (val) { selectedPathId = parseInt(val); await loadPathDetail(selectedPathId); }
    else { selectedPathId = null; pathWaypoints = []; render(); }
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
    <span class="map-legend-item"><span class="map-legend-dot" style="background:#ffa000"></span>Walls</span>
    <span class="map-legend-item"><span class="map-legend-dot" style="background:#8b4513"></span>Obstacles</span>
    <span class="map-legend-item"><span class="map-legend-dot" style="background:#fa4d56;opacity:0.5"></span>Trained</span>
  `;
  container.appendChild(legend);

  refreshPathList();
  if (pollTimer) clearInterval(pollTimer);
  pollData();
  pollTimer = setInterval(pollData, 500);
}

export function destroyMap(): void {
  if (pollTimer) { clearInterval(pollTimer); pollTimer = null; }
  trainingActive = false;
  mapMode = 'navigate';
  drawStart = null;
  lastMouse = null;
}
