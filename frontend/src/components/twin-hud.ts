// HUD overlay — position, heading, speed, mode, connection status

import { onTwinStateChange, getTwinState } from '../state/twin-state';
import type { TwinState } from '../types/twin';

let containerEl: HTMLElement | null = null;
let unsubscribe: (() => void) | null = null;

export function initTwinHud(parent: HTMLElement) {
  if (containerEl) return;

  containerEl = document.createElement('div');
  containerEl.className = 'twin-hud';
  containerEl.innerHTML = `
    <div class="hud-position">
      <span class="hud-label">POS</span>
      <span class="hud-value" id="hud-pos-x">0.00</span>
      <span class="hud-value" id="hud-pos-y">0.00</span>
    </div>
    <div class="hud-heading">
      <span class="hud-label">HDG</span>
      <span class="hud-value" id="hud-heading">0°</span>
    </div>
    <div class="hud-mode">
      <span class="hud-label">MODE</span>
      <span class="hud-value" id="hud-mode">IDLE</span>
    </div>
    <div class="hud-status">
      <span class="status-dot disconnected" id="hud-conn"></span>
      <span class="hud-value" id="hud-status-text">Disconnected</span>
    </div>
  `;
  parent.appendChild(containerEl);

  unsubscribe = onTwinStateChange(updateHud);
}

export function destroyTwinHud() {
  if (unsubscribe) {
    unsubscribe();
    unsubscribe = null;
  }
  if (containerEl) {
    containerEl.remove();
    containerEl = null;
  }
}

function updateHud(state: TwinState) {
  const px = document.getElementById('hud-pos-x');
  const py = document.getElementById('hud-pos-y');
  const hdg = document.getElementById('hud-heading');
  const mode = document.getElementById('hud-mode');
  const conn = document.getElementById('hud-conn');
  const statusText = document.getElementById('hud-status-text');

  if (px) px.textContent = (state.position.x * 0.001).toFixed(2) + 'm';
  if (py) py.textContent = (state.position.y * 0.001).toFixed(2) + 'm';
  if (hdg) hdg.textContent = Math.round(state.heading) + '°';
  if (mode) {
    mode.textContent = state.mode.toUpperCase();
    mode.className = `hud-value hud-mode-${state.mode}`;
  }
  if (conn) {
    conn.className = `status-dot ${state.connected ? 'connected' : 'disconnected'}`;
  }
  if (statusText) {
    statusText.textContent = state.connected ? 'Connected' : 'Disconnected';
  }
}
