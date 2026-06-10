// System status component

import { fetchSystemStatus, fetchSensors } from '../api';

let pollInterval: ReturnType<typeof setInterval> | null = null;

export function initSystem(container: HTMLElement) {
  container.innerHTML = `
    <div class="page-header">
      <h2>System</h2>
      <button class="btn btn--secondary" id="btn-refresh-system">
        <span class="ph ph-arrows-clockwise"></span> Refresh
      </button>
    </div>

    <div class="grid grid--2">
      <div class="card">
        <div class="card-header"><h3>Components</h3></div>
        <div class="card-body">
          <table class="data-table">
            <thead><tr><th>Component</th><th>Status</th><th>Detail</th></tr></thead>
            <tbody>
              <tr><td>Arduino Mega</td><td id="sys-mega-status">--</td><td id="sys-mega-detail">--</td></tr>
              <tr><td>ROS2</td><td id="sys-ros2-status">--</td><td id="sys-ros2-detail">--</td></tr>
              <tr><td>Sensors</td><td id="sys-sensor-status">--</td><td id="sys-sensor-detail">--</td></tr>
              <tr><td>Mode</td><td id="sys-mode">--</td><td></td></tr>
            </tbody>
          </table>
        </div>
      </div>

      <div class="card">
        <div class="card-header"><h3>Sensor Summary</h3></div>
        <div class="card-body">
          <table class="data-table">
            <thead><tr><th>Sensor</th><th>Value</th></tr></thead>
            <tbody id="sys-sensor-table"></tbody>
          </table>
        </div>
      </div>
    </div>

    <div class="card" style="margin-top:1rem">
      <div class="card-header"><h3>Endpoints</h3></div>
      <div class="card-body">
        <table class="data-table">
          <thead><tr><th>Method</th><th>Path</th><th>Description</th></tr></thead>
          <tbody>
            <tr><td><span class="tag tag--green">GET</span></td><td class="mono">/api/sensors</td><td>Current sensor readings</td></tr>
            <tr><td><span class="tag tag--green">GET</span></td><td class="mono">/api/status</td><td>System status</td></tr>
            <tr><td><span class="tag tag--blue">POST</span></td><td class="mono">/api/command</td><td>Send robot command</td></tr>
            <tr><td><span class="tag tag--green">GET</span></td><td class="mono">/api/automations</td><td>List automations</td></tr>
            <tr><td><span class="tag tag--blue">POST</span></td><td class="mono">/api/automations</td><td>Create automation</td></tr>
            <tr><td><span class="tag tag--yellow">PUT</span></td><td class="mono">/api/automations/:id</td><td>Update automation</td></tr>
            <tr><td><span class="tag tag--red">DELETE</span></td><td class="mono">/api/automations/:id</td><td>Delete automation</td></tr>
            <tr><td><span class="tag tag--blue">POST</span></td><td class="mono">/api/automations/:id/toggle</td><td>Enable/disable</td></tr>
            <tr><td><span class="tag tag--blue">POST</span></td><td class="mono">/api/automations/:id/run</td><td>Trigger manually</td></tr>
            <tr><td><span class="tag tag--green">GET</span></td><td class="mono">/api/feeds</td><td>All sensor feed values</td></tr>
          </tbody>
        </table>
      </div>
    </div>
  `;

  container.querySelector('#btn-refresh-system')?.addEventListener('click', refreshSystem);
  refreshSystem();
  pollInterval = setInterval(refreshSystem, 2000);
}

export function destroySystem() {
  if (pollInterval) {
    clearInterval(pollInterval);
    pollInterval = null;
  }
}

async function refreshSystem() {
  try {
    const status = await fetchSystemStatus();
    const el = (id: string) => document.getElementById(id);
    el('sys-mega-status')!.innerHTML = status.mega_connected
      ? '<span class="tag tag--green">Connected</span>'
      : '<span class="tag tag--red">Disconnected</span>';
    el('sys-ros2-status')!.innerHTML = status.ros2_available
      ? '<span class="tag tag--green">Available</span>'
      : '<span class="tag tag--gray">Unavailable</span>';
    el('sys-mode')!.textContent = status.simulation_mode ? 'Simulation' : 'Hardware';
  } catch { /* silent */ }

  try {
    const sensors = await fetchSensors();
    const tbody = document.getElementById('sys-sensor-table');
    if (tbody) {
      const entries = Object.entries(sensors).filter(([k]) => k !== 'mega_connected');
      tbody.innerHTML = entries
        .map(([k, v]) => `<tr><td>${k}</td><td class="mono">${typeof v === 'number' ? v.toFixed(2) : v}</td></tr>`)
        .join('');
    }
  } catch { /* silent */ }
}
