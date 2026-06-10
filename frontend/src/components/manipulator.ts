// Manipulator control — gripper, lifter, camera tilt

import { sendMovementCommand } from '../api';

export function initManipulator(container: HTMLElement) {
  container.innerHTML = `
    <div class="page-header">
      <h2>Manipulator</h2>
    </div>

    <div class="grid grid--3">
      <div class="card">
        <div class="card-header"><h3>Gripper</h3></div>
        <div class="card-body">
          <div class="control-group">
            <button class="btn btn--secondary btn--block" data-cmd="no">Open</button>
            <button class="btn btn--secondary btn--block" data-cmd="nh">Half</button>
            <button class="btn btn--primary btn--block" data-cmd="nc">Close</button>
          </div>
        </div>
      </div>

      <div class="card">
        <div class="card-header"><h3>Lifter</h3></div>
        <div class="card-body">
          <div class="control-group">
            <button class="btn btn--primary btn--block" data-cmd="u">
              <span class="ph ph-arrow-up"></span> Up
            </button>
            <button class="btn btn--danger btn--block" data-cmd="s">Stop</button>
            <button class="btn btn--secondary btn--block" data-cmd="d">
              <span class="ph ph-arrow-down"></span> Down
            </button>
          </div>
        </div>
      </div>

      <div class="card">
        <div class="card-header"><h3>Camera Tilt</h3></div>
        <div class="card-body">
          <div class="control-group">
            <button class="btn btn--secondary btn--block" data-cmd="mu">
              <span class="ph ph-arrow-up"></span> Tilt Up
            </button>
            <button class="btn btn--secondary btn--block" data-cmd="mc">Center</button>
            <button class="btn btn--secondary btn--block" data-cmd="md">
              <span class="ph ph-arrow-down"></span> Tilt Down
            </button>
          </div>
          <div class="tilt-angle" style="margin-top: 1rem;">
            <label class="label">Angle (0-180)</label>
            <div class="input-row">
              <input type="range" id="tilt-angle" min="0" max="180" value="90" class="input-range">
              <span id="tilt-angle-val" class="mono">90</span>
              <button class="btn btn--primary btn--sm" id="tilt-angle-set">Set</button>
            </div>
          </div>
        </div>
      </div>
    </div>
  `;

  // Button clicks
  container.querySelectorAll('[data-cmd]').forEach((btn) => {
    btn.addEventListener('click', () => {
      const cmd = (btn as HTMLElement).dataset.cmd;
      if (cmd) sendMovementCommand(cmd);
    });
  });

  // Tilt angle slider
  const slider = container.querySelector('#tilt-angle') as HTMLInputElement;
  const valLabel = container.querySelector('#tilt-angle-val') as HTMLElement;
  const setBtn = container.querySelector('#tilt-angle-set') as HTMLElement;

  slider?.addEventListener('input', () => {
    valLabel.textContent = slider.value;
  });

  setBtn?.addEventListener('click', () => {
    const angle = parseInt(slider.value, 10);
    if (!isNaN(angle)) {
      sendMovementCommand(`ta${angle}`);
    }
  });
}

export function destroyManipulator() {}
