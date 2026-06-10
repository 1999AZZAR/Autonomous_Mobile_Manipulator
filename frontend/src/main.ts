// Main entry point — wires tabs, WebSocket, and components

import { SensorSocket } from './websocket';
import { initDashboard, destroyDashboard } from './components/dashboard';
import { initMovement, destroyMovement } from './components/movement';
import { initManipulator, destroyManipulator } from './components/manipulator';
import { initAutomations, destroyAutomations } from './components/automations';
import { renderAiControl } from './components/ai-control';
import { renderMap, destroyMap } from './components/map-view';
import { initSystem, destroySystem } from './components/system';
import { emergencyStop } from './api';

const socket = new SensorSocket();

const TABS = ['dashboard', 'movement', 'manipulator', 'map', 'automations', 'ai', 'system'] as const;
type Tab = (typeof TABS)[number];

const initFns: Record<Tab, (el: HTMLElement) => void> = {
  dashboard: initDashboard,
  movement: initMovement,
  manipulator: initManipulator,
  map: renderMap,
  automations: initAutomations,
  ai: renderAiControl,
  system: initSystem,
};

const destroyFns: Record<Tab, () => void> = {
  dashboard: destroyDashboard,
  movement: destroyMovement,
  manipulator: destroyManipulator,
  map: destroyMap,
  automations: destroyAutomations,
  ai: () => {},
  system: destroySystem,
};

let activeTab: Tab = 'dashboard';
const initialized = new Set<Tab>();

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

  // init once, then just toggle visibility
  if (!initialized.has(tab) && panel) {
    initFns[tab](panel);
    initialized.add(tab);
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
  initialized.add(activeTab);
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
