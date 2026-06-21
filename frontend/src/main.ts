// Main entry point — wires tabs, WebSocket, and components

import { SensorSocket } from './websocket';
import { initDashboard, destroyDashboard } from './components/dashboard';
import { initMovement, destroyMovement } from './components/movement';
import { initManipulator, destroyManipulator } from './components/manipulator';
import { initAutomations, destroyAutomations } from './components/automations';
import { renderAiControl, destroyAiControl } from './components/ai-control';
import { renderMap, destroyMap } from './components/map-view';
import { initDigitalTwin, destroyDigitalTwin } from './components/digital-twin';
import { initTwinHud, destroyTwinHud } from './components/twin-hud';
import { initTwinControls, destroyTwinControls } from './components/twin-controls';
import { initSystem, destroySystem } from './components/system';
import { emergencyStop } from './api';

const socket = new SensorSocket();

const TABS = ['dashboard', 'movement', 'manipulator', 'twin', 'automations', 'ai', 'system'] as const;
type Tab = (typeof TABS)[number];

const initFns: Record<Tab, (el: HTMLElement) => void> = {
  dashboard: initDashboard,
  movement: initMovement,
  manipulator: initManipulator,
  twin: initTwinTab,
  automations: initAutomations,
  ai: renderAiControl,
  system: initSystem,
};

const destroyFns: Record<Tab, () => void> = {
  dashboard: destroyDashboard,
  movement: destroyMovement,
  manipulator: destroyManipulator,
  twin: destroyTwinTab,
  automations: destroyAutomations,
  ai: destroyAiControl,
  system: destroySystem,
};

let activeTab: Tab = 'dashboard';

function initTwinTab(el: HTMLElement) {
  el.innerHTML = `
    <div class="twin-layout">
      <div class="twin-viewport" id="twin-viewport"></div>
      <div class="twin-sidebar" id="twin-sidebar"></div>
    </div>
    <div class="twin-map-panel" id="twin-map-panel" style="display:none"></div>
  `;
  const viewport = document.getElementById('twin-viewport');
  const sidebar = document.getElementById('twin-sidebar');
  if (viewport) initDigitalTwin(viewport);
  if (sidebar) {
    initTwinHud(sidebar);
    initTwinControls(sidebar);
  }
}

function destroyTwinTab() {
  destroyDigitalTwin();
  destroyTwinHud();
  destroyTwinControls();
  destroyMap();
}

function switchTab(tab: Tab) {
  if (tab === activeTab) return;

  // destroy old
  destroyFns[activeTab]();

  // hide all panels
  TABS.forEach((t) => {
    const panel = document.getElementById(`tab-${t}`);
    if (panel) panel.classList.remove('active');
  });

  // deactivate nav buttons
  document.querySelectorAll('.nav-item').forEach((btn) => btn.classList.remove('active'));

  // activate new
  const panel = document.getElementById(`tab-${tab}`);
  if (panel) panel.classList.add('active');

  const navBtn = document.querySelector(`.nav-item[data-tab="${tab}"]`);
  if (navBtn) navBtn.classList.add('active');

  activeTab = tab;

  // init on every re-entry (destroyFns tear down resources)
  if (panel) {
    initFns[tab](panel);
  }
}

// Wire nav buttons
document.querySelectorAll('.nav-item').forEach((btn) => {
  btn.addEventListener('click', () => {
    const tab = (btn as HTMLElement).dataset.tab as Tab;
    if (tab && TABS.includes(tab)) switchTab(tab);
  });
});

// Init default tab
const defaultPanel = document.getElementById(`tab-${activeTab}`);
if (defaultPanel) {
  initFns[activeTab](defaultPanel);
}

// Connect WebSocket
socket.connect();

// Emergency stop button
const estopBtn = document.getElementById('emergency-stop');
if (estopBtn) {
  estopBtn.addEventListener('click', async () => {
    estopBtn.classList.add('active');
    try {
      await emergencyStop();
    } catch {
      // Best effort
    }
    setTimeout(() => estopBtn.classList.remove('active'), 1000);
  });
}
