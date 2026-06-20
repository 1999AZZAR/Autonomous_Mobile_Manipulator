// AI Decision Engine control panel + waypoint manager

import { fetchAiStatus, startAiLoop, stopAiLoop, analyzeOnce, fetchAiDecisions, sendHumanGuidance, updateAiConfig, getCameraSnapshotUrl, fetchPaths, createPath, deletePath, recordWaypoint, stopRecording, startReplay, stopReplay, fetchWaypointStatus, fetchPath } from '../api';
import type { AiStatus, AiDecision, SavedPath, WaypointStatus } from '../types';

let pollTimer: ReturnType<typeof setInterval> | null = null;

function el(tag: string, cls?: string, text?: string): HTMLElement {
  const e = document.createElement(tag);
  if (cls) e.className = cls;
  if (text) e.textContent = text;
  return e;
}

function confidenceColor(c: number | null): string {
  if (c === null) return 'var(--text-muted)';
  if (c >= 0.7) return 'var(--color-success, #4caf50)';
  if (c >= 0.4) return 'var(--color-warning, #ff9800)';
  return 'var(--color-danger, #f44336)';
}

function modeIcon(mode: string): string {
  switch (mode) {
    case 'ai': return 'brain';
    case 'replay': return 'route';
    case 'ifttt': return 'logic';
    default: return 'circle';
  }
}

export function renderAiControl(container: HTMLElement): void {
  container.innerHTML = '';
  container.appendChild(el('h2', '', 'AI Decision Engine'));

  // Main grid: camera + controls
  const grid = el('div', 'ai-grid');

  // Left: camera preview
  const camPanel = el('div', 'ai-panel ai-camera-panel');
  camPanel.appendChild(el('h3', '', 'Camera'));
  const preview = el('div', 'ai-camera-preview');
  const camImg = document.createElement('img');
  camImg.alt = 'Camera preview';
  camImg.style.maxWidth = '100%';
  camImg.style.borderRadius = '8px';
  camImg.id = 'ai-cam-img';
  preview.appendChild(camImg);
  const camStatus = el('div', 'ai-cam-status', 'Camera OFF');
  camStatus.id = 'ai-cam-status';
  preview.appendChild(camStatus);
  camPanel.appendChild(preview);
  grid.appendChild(camPanel);

  // Right: controls
  const ctrlPanel = el('div', 'ai-panel ai-ctrl-panel');

  // Status row
  const statusRow = el('div', 'ai-status-row');
  const modeIndicator = el('span', 'ai-mode-badge', 'Mode: ---');
  modeIndicator.id = 'ai-mode-badge';
  const confIndicator = el('span', 'ai-conf-badge', 'Confidence: --');
  confIndicator.id = 'ai-conf-badge';
  statusRow.appendChild(modeIndicator);
  statusRow.appendChild(confIndicator);
  ctrlPanel.appendChild(statusRow);

  // Task goal
  ctrlPanel.appendChild(el('h3', '', 'Task Goal'));
  const goalInput = document.createElement('textarea');
  goalInput.id = 'ai-goal-input';
  goalInput.placeholder = 'e.g. Navigate to the kitchen and pick up the red cup';
  goalInput.rows = 3;
  ctrlPanel.appendChild(goalInput);

  // Interval + backend config
  const cfgRow = el('div', 'ai-cfg-row');
  const intervalInput = document.createElement('input');
  intervalInput.type = 'number';
  intervalInput.id = 'ai-interval-input';
  intervalInput.value = '3';
  intervalInput.min = '1';
  intervalInput.max = '30';
  intervalInput.placeholder = 'Interval (s)';
  cfgRow.appendChild(intervalInput);

  const backendSelect = document.createElement('select');
  backendSelect.id = 'ai-backend-select';
  ['hybrid', 'local', 'api'].forEach(opt => {
    const o = document.createElement('option');
    o.value = opt;
    o.textContent = opt;
    backendSelect.appendChild(o);
  });
  cfgRow.appendChild(backendSelect);
  ctrlPanel.appendChild(cfgRow);

  // Buttons
  const btnRow = el('div', 'ai-btn-row');
  const startBtn = el('button', 'btn btn-primary', 'Start Loop');
  startBtn.onclick = async () => {
    const goal = goalInput.value.trim();
    const interval = parseFloat(intervalInput.value) || 3;
    await startAiLoop(goal, interval);
    await updateAiConfig({ backend: backendSelect.value });
    refreshStatus();
  };
  const stopBtn = el('button', 'btn btn-danger', 'Stop');
  stopBtn.onclick = async () => { await stopAiLoop(); refreshStatus(); };
  const analyzeBtn = el('button', 'btn btn-secondary', 'Analyze Once');
  analyzeBtn.onclick = async () => {
    const goal = goalInput.value.trim();
    await analyzeOnce(goal);
    refreshDecisions();
  };
  btnRow.appendChild(startBtn);
  btnRow.appendChild(stopBtn);
  btnRow.appendChild(analyzeBtn);
  ctrlPanel.appendChild(btnRow);

  // Human guidance
  ctrlPanel.appendChild(el('h3', '', 'Human Guidance'));
  const guidanceInput = document.createElement('textarea');
  guidanceInput.id = 'ai-guidance-input';
  guidanceInput.placeholder = 'e.g. Avoid the chair on the left';
  guidanceInput.rows = 2;
  ctrlPanel.appendChild(guidanceInput);
  const guidanceBtn = el('button', 'btn btn-secondary', 'Send Guidance');
  guidanceBtn.onclick = async () => {
    const g = guidanceInput.value.trim();
    if (g) { await sendHumanGuidance(g); guidanceInput.value = ''; }
  };
  ctrlPanel.appendChild(guidanceBtn);

  grid.appendChild(ctrlPanel);
  container.appendChild(grid);

  // Decision history
  container.appendChild(el('h2', '', 'Decision History'));
  const historyList = el('div', 'ai-history-list');
  historyList.id = 'ai-history-list';
  container.appendChild(historyList);

  // Waypoint manager
  container.appendChild(el('h2', '', 'Waypoint Memory'));
  renderWaypointManager(container);

  // Start polling
  refreshStatus();
  refreshDecisions();
  if (pollTimer) clearInterval(pollTimer);
  pollTimer = setInterval(() => {
    refreshStatus();
    refreshDecisions();
  }, 3000);
}

async function refreshStatus(): Promise<void> {
  try {
    const [status, wpStatus] = await Promise.all([
      fetchAiStatus(),
      fetchWaypointStatus().catch(() => null),
    ]);
    const badge = document.getElementById('ai-mode-badge');
    const conf = document.getElementById('ai-conf-badge');
    const camStatus = document.getElementById('ai-cam-status');
    const camImg = document.getElementById('ai-cam-img') as HTMLImageElement;
    const stopReplayBtn = document.getElementById('ai-wp-stop-replay');

    if (badge) badge.textContent = `Mode: ${status.mode.toUpperCase()}`;
    if (conf) {
      const c = status.last_decision?.confidence;
      conf.textContent = `Confidence: ${c != null ? Math.round(c * 100) + '%' : '--'}`;
      conf.style.color = confidenceColor(c ?? null);
    }
    if (camStatus) {
      camStatus.textContent = `Camera: ${status.camera.state.toUpperCase()}`;
      camStatus.style.color = status.camera.state === 'active' ? 'var(--color-success, #4caf50)' : 'var(--text-muted)';
    }
    if (camImg && status.camera.state === 'active') {
      camImg.src = `${getCameraSnapshotUrl()}?t=${Date.now()}`;
    } else if (camImg) {
      camImg.src = '';
    }

    // Stop replay button visibility
    if (stopReplayBtn) {
      if (wpStatus && wpStatus.replaying) {
        stopReplayBtn.style.display = '';
      } else {
        stopReplayBtn.style.display = 'none';
      }
    }
  } catch {
    // Engine not available
  }
}

async function refreshDecisions(): Promise<void> {
  const list = document.getElementById('ai-history-list');
  if (!list) return;

  try {
    const { decisions } = await fetchAiDecisions(10);
    list.innerHTML = '';
    if (decisions.length === 0) {
      list.appendChild(el('div', 'ai-history-empty', 'No decisions yet. Start the AI loop or run Analyze Once.'));
      return;
    }
    decisions.forEach(d => {
      const item = el('div', 'ai-history-item');
      const time = d.createdAt ? new Date(d.createdAt).toLocaleTimeString() : '--:--';
      const reasoning = d.aiResponse?.reasoning || 'No reasoning';
      const actions = d.aiResponse?.actions?.map(a => `${a.type}=${a.value}`).join(', ') || 'none';
      const conf = d.confidence != null ? Math.round(d.confidence * 100) + '%' : '--';

      item.innerHTML = `
        <div class="ai-history-header">
          <span class="ai-history-time">${time}</span>
          <span class="ai-history-mode">${d.mode}</span>
          <span class="ai-history-conf" style="color:${confidenceColor(d.confidence)}">${conf}</span>
        </div>
        <div class="ai-history-reasoning">${reasoning}</div>
        <div class="ai-history-actions">Actions: ${actions}</div>
      `;
      list.appendChild(item);
    });
  } catch {
    list.innerHTML = '';
    list.appendChild(el('div', 'ai-history-empty', 'AI engine not available'));
  }
}

function renderWaypointManager(container: HTMLElement): void {
  const panel = el('div', 'ai-panel ai-waypoint-panel');

  // Recording controls
  const recRow = el('div', 'ai-wp-rec-row');
  const nameInput = document.createElement('input');
  nameInput.type = 'text';
  nameInput.id = 'ai-wp-name';
  nameInput.placeholder = 'Path name';
  recRow.appendChild(nameInput);

  const recBtn = el('button', 'btn btn-primary', 'Record Path');
  recBtn.id = 'ai-wp-rec-btn';
  recBtn.onclick = async () => {
    const name = nameInput.value.trim();
    if (!name) return;
    const status = await fetchWaypointStatus();
    if (status.recording) {
      await stopRecording();
      recBtn.textContent = 'Record Path';
    } else {
      await createPath(name);
      recBtn.textContent = 'Stop Recording';
    }
  };
  recRow.appendChild(recBtn);

  const addWpBtn = el('button', 'btn btn-secondary', '+ Waypoint');
  addWpBtn.onclick = async () => { await recordWaypoint(); };
  recRow.appendChild(addWpBtn);

  const stopReplayBtn = el('button', 'btn btn-danger btn-sm', 'Stop Replay');
  stopReplayBtn.style.display = 'none';
  stopReplayBtn.id = 'ai-wp-stop-replay';
  stopReplayBtn.onclick = async () => { await stopReplay(); };
  recRow.appendChild(stopReplayBtn);

  panel.appendChild(recRow);

  // Path list
  const pathList = el('div', 'ai-wp-paths');
  pathList.id = 'ai-wp-paths';
  panel.appendChild(pathList);
  container.appendChild(panel);

  refreshPaths();
}

export function destroyAiControl() {
  if (pollTimer) {
    clearInterval(pollTimer);
    pollTimer = null;
  }
}

async function refreshPaths(): Promise<void> {
  const list = document.getElementById('ai-wp-paths');
  if (!list) return;

  try {
    const { paths } = await fetchPaths();
    list.innerHTML = '';
    if (paths.length === 0) {
      list.appendChild(el('div', 'ai-wp-empty', 'No saved paths. Record your first path above.'));
      return;
    }
    paths.forEach(p => {
      const item = el('div', 'ai-wp-item');
      const nameSpan = el('span', 'ai-wp-name', p.name);
      const countSpan = el('span', 'ai-wp-count', `${p.waypoint_count} waypoints`);
      const replayBtn = el('button', 'btn btn-sm btn-primary', 'Replay');
      replayBtn.onclick = async () => { await startReplay(p.id); };
      const delBtn = el('button', 'btn btn-sm btn-danger', 'Delete');
      delBtn.onclick = async () => { await deletePath(p.id); refreshPaths(); };
      item.appendChild(nameSpan);
      item.appendChild(countSpan);
      item.appendChild(replayBtn);
      item.appendChild(delBtn);
      list.appendChild(item);
    });
  } catch {
    list.innerHTML = '';
    list.appendChild(el('div', 'ai-wp-empty', 'Waypoint system not available'));
  }
}
