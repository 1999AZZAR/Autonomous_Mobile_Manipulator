// Dashboard component

import type { SensorData, SystemStatus } from '../types';
import { fetchSensors, fetchSystemStatus } from '../api';

let sensorInterval: ReturnType<typeof setInterval> | null = null;

export function initDashboard(container: HTMLElement) {
  container.innerHTML = `
    <div class="page-header">
      <h2>Dashboard</h2>
      <span class="tag tag--green">Live</span>
    </div>

    <div class="kpi-strip" id="kpi-strip">
      <div class="kpi-card">
        <span class="kpi-label">Heading</span>
        <span class="kpi-value" id="kpi-heading">--</span>
        <span class="kpi-unit">deg</span>
      </div>
      <div class="kpi-card">
        <span class="kpi-label">Front Left</span>
        <span class="kpi-value" id="kpi-ultra-fl">--</span>
        <span class="kpi-unit">cm</span>
      </div>
      <div class="kpi-card">
        <span class="kpi-label">Front Right</span>
        <span class="kpi-value" id="kpi-ultra-fr">--</span>
        <span class="kpi-unit">cm</span>
      </div>
      <div class="kpi-card">
        <span class="kpi-label">TF-Luna</span>
        <span class="kpi-value" id="kpi-tfluna">--</span>
        <span class="kpi-unit">cm</span>
      </div>
      <div class="kpi-card">
        <span class="kpi-label">Mega</span>
        <span class="kpi-value" id="kpi-mega">--</span>
        <span class="kpi-unit"></span>
      </div>
    </div>

    <div class="grid grid--2">
      <div class="card">
        <div class="card-header">
          <h3>Laser Sensors</h3>
        </div>
        <div class="card-body">
          <table class="data-table">
            <thead>
              <tr><th>Position</th><th>Value</th></tr>
            </thead>
            <tbody id="laser-table">
              <tr><td>Left Front</td><td id="laser-lf">--</td></tr>
              <tr><td>Left Back</td><td id="laser-lb">--</td></tr>
              <tr><td>Right Front</td><td id="laser-rf">--</td></tr>
              <tr><td>Right Back</td><td id="laser-rb">--</td></tr>
              <tr><td>Back Left</td><td id="laser-bl">--</td></tr>
              <tr><td>Back Right</td><td id="laser-br">--</td></tr>
            </tbody>
          </table>
        </div>
      </div>

      <div class="card">
        <div class="card-header">
          <h3>Line Sensors</h3>
        </div>
        <div class="card-body">
          <div class="line-sensor-grid">
            <div class="line-sensor" id="line-left">
              <span class="line-dot"></span>
              <span>Left</span>
            </div>
            <div class="line-sensor" id="line-center">
              <span class="line-dot"></span>
              <span>Center</span>
            </div>
            <div class="line-sensor" id="line-right">
              <span class="line-dot"></span>
              <span>Right</span>
            </div>
          </div>
        </div>
      </div>

      <div class="card">
        <div class="card-header">
          <h3>IMU</h3>
        </div>
        <div class="card-body">
          <table class="data-table">
            <thead>
              <tr><th>Axis</th><th>Value</th></tr>
            </thead>
            <tbody>
              <tr><td>Heading (Yaw)</td><td id="imu-heading">--</td></tr>
              <tr><td>Pitch</td><td id="imu-pitch">--</td></tr>
              <tr><td>Roll</td><td id="imu-roll">--</td></tr>
            </tbody>
          </table>
        </div>
      </div>

      <div class="card">
        <div class="card-header">
          <h3>System</h3>
        </div>
        <div class="card-body">
          <table class="data-table">
            <thead>
              <tr><th>Component</th><th>Status</th></tr>
            </thead>
            <tbody id="system-status-table">
              <tr><td>Mega</td><td id="status-mega">--</td></tr>
              <tr><td>ROS2</td><td id="status-ros2">--</td></tr>
              <tr><td>Mode</td><td id="status-mode">--</td></tr>
            </tbody>
          </table>
        </div>
      </div>
    </div>
  `;

  // Initial fetch
  refreshDashboard();

  // Poll every 500ms
  sensorInterval = setInterval(refreshDashboard, 500);
}

export function destroyDashboard() {
  if (sensorInterval) {
    clearInterval(sensorInterval);
    sensorInterval = null;
  }
}

async function refreshDashboard() {
  try {
    const sensors = await fetchSensors();
    updateSensorDisplay(sensors);
  } catch { /* silent */ }

  try {
    const status = await fetchSystemStatus();
    updateSystemStatus(status);
  } catch { /* silent */ }
}

function updateSensorDisplay(d: SensorData) {
  setText('kpi-heading', d.imu_heading?.toFixed(1) ?? '--');
  setText('kpi-ultra-fl', d.ultra_front_left?.toFixed(0) ?? '--');
  setText('kpi-ultra-fr', d.ultra_front_right?.toFixed(0) ?? '--');
  setText('kpi-tfluna', d.tf_luna_distance?.toFixed(0) ?? '--');
  setText('kpi-mega', d.mega_connected ? 'ON' : 'OFF');

  setText('laser-lf', d.laser_left_front?.toFixed(1) ?? '--');
  setText('laser-lb', d.laser_left_back?.toFixed(1) ?? '--');
  setText('laser-rf', d.laser_right_front?.toFixed(1) ?? '--');
  setText('laser-rb', d.laser_right_back?.toFixed(1) ?? '--');
  setText('laser-bl', d.laser_back_left?.toFixed(1) ?? '--');
  setText('laser-br', d.laser_back_right?.toFixed(1) ?? '--');

  setLineColor('line-left', d.line_left);
  setLineColor('line-center', d.line_center);
  setLineColor('line-right', d.line_right);

  setText('imu-heading', d.imu_heading?.toFixed(1) ?? '--');
  setText('imu-pitch', d.imu_pitch?.toFixed(1) ?? '--');
  setText('imu-roll', d.imu_roll?.toFixed(1) ?? '--');
}

function updateSystemStatus(s: SystemStatus) {
  setText('status-mega', s.mega_connected ? 'Connected' : 'Disconnected');
  setText('status-ros2', s.ros2_available ? 'Available' : 'Unavailable');
  setText('status-mode', s.simulation_mode ? 'Simulation' : 'Hardware');
}

function setText(id: string, val: string) {
  const el = document.getElementById(id);
  if (el) el.textContent = val;
}

function setLineColor(id: string, val: number) {
  const el = document.getElementById(id);
  if (!el) return;
  const dot = el.querySelector('.line-dot');
  if (dot) {
    dot.className = `line-dot ${val > 0.5 ? 'active' : ''}`;
  }
}
