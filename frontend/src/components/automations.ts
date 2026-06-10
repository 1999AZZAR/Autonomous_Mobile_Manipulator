// Automations component — CRUD list, create modal, conditions/actions builder

import type { Automation, AutomationCondition, AutomationAction } from '../types';
import {
  fetchAutomations,
  createAutomation,
  deleteAutomation,
  toggleAutomation,
  runAutomation,
  fetchAutomationLogs,
} from '../api';

let listEl: HTMLElement | null = null;

const FEED_KEYS = [
  'laser_left_front', 'laser_left_back', 'laser_right_front', 'laser_right_back',
  'laser_back_left', 'laser_back_right',
  'ultra_front_left', 'ultra_front_right',
  'line_left', 'line_center', 'line_right',
  'imu_heading', 'imu_pitch', 'imu_roll',
  'tf_luna_distance',
];

const OPERATORS = ['>', '<', '==', '!=', '>=', '<='];
const ACTION_TYPES = ['move', 'turn', 'gripper', 'lifter', 'tilt', 'stop', 'speed', 'delay', 'webhook'];

export function initAutomations(container: HTMLElement) {
  listEl = container;
  container.innerHTML = `
    <div class="page-header">
      <h2>Automations</h2>
      <button class="btn btn--primary" id="btn-create-auto">
        <span class="ph ph-plus"></span> New Automation
      </button>
    </div>
    <div id="auto-list" class="auto-list"></div>

    <div class="modal-overlay" id="auto-modal" style="display:none">
      <div class="modal">
        <div class="modal-header">
          <h3>New Automation</h3>
          <button class="btn-close" id="modal-close"><span class="ph ph-x"></span></button>
        </div>
        <div class="modal-body">
          <div class="form-group">
            <label class="label">Name</label>
            <input type="text" id="auto-name" class="input" placeholder="My Automation">
          </div>
          <div class="form-group">
            <label class="label">Description</label>
            <input type="text" id="auto-desc" class="input" placeholder="Optional description">
          </div>
          <div class="form-group">
            <label class="label">Trigger Type</label>
            <select id="auto-trigger" class="input">
              <option value="sensor">Sensor</option>
              <option value="time">Time</option>
              <option value="manual">Manual</option>
            </select>
          </div>

          <h4 style="margin-top:1.5rem;margin-bottom:.5rem;">Conditions</h4>
          <div id="conditions-builder"></div>
          <button class="btn btn--secondary btn--sm" id="add-condition" style="margin-top:.5rem">
            <span class="ph ph-plus"></span> Add Condition
          </button>

          <h4 style="margin-top:1.5rem;margin-bottom:.5rem;">Actions</h4>
          <div id="actions-builder"></div>
          <button class="btn btn--secondary btn--sm" id="add-action" style="margin-top:.5rem">
            <span class="ph ph-plus"></span> Add Action
          </button>
        </div>
        <div class="modal-footer">
          <button class="btn btn--secondary" id="modal-cancel">Cancel</button>
          <button class="btn btn--primary" id="modal-save">Save</button>
        </div>
      </div>
    </div>
  `;

  container.querySelector('#btn-create-auto')?.addEventListener('click', openModal);
  container.querySelector('#modal-close')?.addEventListener('click', closeModal);
  container.querySelector('#modal-cancel')?.addEventListener('click', closeModal);
  container.querySelector('#add-condition')?.addEventListener('click', addConditionRow);
  container.querySelector('#add-action')?.addEventListener('click', addActionRow);
  container.querySelector('#modal-save')?.addEventListener('click', saveAutomation);

  loadAutomations();
}

function openModal() {
  const modal = listEl?.querySelector('#auto-modal') as HTMLElement;
  if (modal) modal.style.display = 'flex';
}

function closeModal() {
  const modal = listEl?.querySelector('#auto-modal') as HTMLElement;
  if (modal) modal.style.display = 'none';
}

function addConditionRow() {
  const builder = listEl?.querySelector('#conditions-builder');
  if (!builder) return;
  const row = document.createElement('div');
  row.className = 'condition-row';
  row.innerHTML = `
    <select class="input input--sm">${FEED_KEYS.map((k) => `<option value="${k}">${k}</option>`).join('')}</select>
    <select class="input input--sm">${OPERATORS.map((o) => `<option value="${o}">${o}</option>`).join('')}</select>
    <input type="text" class="input input--sm" placeholder="Threshold">
    <select class="input input--sm"><option value="AND">AND</option><option value="OR">OR</option></select>
    <button class="btn btn--danger btn--sm btn-icon"><span class="ph ph-trash"></span></button>
  `;
  row.querySelector('.btn-icon')?.addEventListener('click', () => row.remove());
  builder.appendChild(row);
}

function addActionRow() {
  const builder = listEl?.querySelector('#actions-builder');
  if (!builder) return;
  const row = document.createElement('div');
  row.className = 'action-row';
  row.innerHTML = `
    <select class="input input--sm action-type">${ACTION_TYPES.map((a) => `<option value="${a}">${a}</option>`).join('')}</select>
    <input type="text" class="input input--sm action-params" placeholder='{"command":"f"}'>
    <input type="number" class="input input--sm action-delay" placeholder="Delay ms" value="0" min="0">
    <button class="btn btn--danger btn--sm btn-icon"><span class="ph ph-trash"></span></button>
  `;
  row.querySelector('.btn-icon')?.addEventListener('click', () => row.remove());
  builder.appendChild(row);
}

async function saveAutomation() {
  const name = (listEl?.querySelector('#auto-name') as HTMLInputElement)?.value;
  const desc = (listEl?.querySelector('#auto-desc') as HTMLInputElement)?.value;
  const triggerType = (listEl?.querySelector('#auto-trigger') as HTMLSelectElement)?.value;

  if (!name) return alert('Name required');

  const condRows = listEl?.querySelectorAll('.condition-row');
  const conditions: Omit<AutomationCondition, 'id' | 'automationId'>[] = [];
  condRows?.forEach((row) => {
    const selects = row.querySelectorAll('select');
    const input = row.querySelector('input');
    conditions.push({
      feedKey: selects[0]?.value ?? '',
      operator: (selects[1]?.value as AutomationCondition['operator']) ?? '>',
      threshold: input?.value ?? '',
      logicGate: (selects[2]?.value as AutomationCondition['logicGate']) ?? 'AND',
      conditionOrder: conditions.length,
    });
  });

  const actRows = listEl?.querySelectorAll('.action-row');
  const actions: Omit<AutomationAction, 'id' | 'automationId'>[] = [];
  actRows?.forEach((row) => {
    const typeEl = row.querySelector('.action-type') as HTMLSelectElement;
    const paramsEl = row.querySelector('.action-params') as HTMLInputElement;
    const delayEl = row.querySelector('.action-delay') as HTMLInputElement;
    actions.push({
      actionType: typeEl?.value ?? '',
      parameters: JSON.parse(paramsEl?.value || '{}'),
      actionOrder: actions.length,
      delayMs: parseInt(delayEl?.value || '0', 10),
    });
  });

  try {
    await createAutomation({ name, description: desc, triggerType, conditions, actions });
    closeModal();
    loadAutomations();
  } catch (err: unknown) {
    alert(`Error: ${err instanceof Error ? err.message : err}`);
  }
}

async function loadAutomations() {
  const listContainer = listEl?.querySelector('#auto-list') as HTMLElement | null;
  if (!listContainer) return;
  try {
    const { automations } = await fetchAutomations();
    renderList(listContainer, automations);
  } catch {
    listContainer.innerHTML = '<p class="muted">Failed to load automations.</p>';
  }
}

function renderList(container: HTMLElement, automations: Automation[]) {
  if (!automations.length) {
    container.innerHTML = '<p class="muted">No automations yet.</p>';
    return;
  }
  container.innerHTML = automations
    .map(
      (a) => `
    <div class="auto-card ${a.enabled ? '' : 'disabled'}">
      <div class="auto-card-header">
        <div>
          <strong>${a.name}</strong>
          <span class="tag tag--${a.triggerType === 'sensor' ? 'blue' : a.triggerType === 'time' ? 'purple' : 'gray'}">${a.triggerType}</span>
        </div>
        <div class="auto-card-actions">
          <button class="btn btn--sm btn--secondary" data-action="toggle" data-id="${a.id}">
            ${a.enabled ? 'Disable' : 'Enable'}
          </button>
          <button class="btn btn--sm btn--primary" data-action="run" data-id="${a.id}">Run</button>
          <button class="btn btn--sm btn--danger" data-action="delete" data-id="${a.id}">
            <span class="ph ph-trash"></span>
          </button>
        </div>
      </div>
      <div class="auto-card-meta">
        <span>${a.conditions?.length ?? 0} conditions</span>
        <span>${a.actions?.length ?? 0} actions</span>
      </div>
    </div>
  `
    )
    .join('');

  container.querySelectorAll('[data-action]').forEach((btn) => {
    btn.addEventListener('click', async () => {
      const id = (btn as HTMLElement).dataset.id;
      const action = (btn as HTMLElement).dataset.action;
      if (!id || !action) return;
      if (action === 'toggle') await toggleAutomation(id);
      else if (action === 'run') await runAutomation(id);
      else if (action === 'delete' && confirm('Delete automation?')) await deleteAutomation(id);
      loadAutomations();
    });
  });
}

export function destroyAutomations() {}
