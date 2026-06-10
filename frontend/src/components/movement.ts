// Movement control component — D-Pad + speed control

import { sendMovementCommand } from '../api';

export function initMovement(container: HTMLElement) {
  container.innerHTML = `
    <div class="page-header">
      <h2>Movement</h2>
    </div>

    <div class="grid grid--2">
      <div class="card">
        <div class="card-header">
          <h3>D-Pad</h3>
        </div>
        <div class="card-body">
          <div class="dpad">
            <button class="dpad-btn dpad-up" data-cmd="f" title="Forward">
              <span class="ph ph-caret-up"></span>
            </button>
            <button class="dpad-btn dpad-left" data-cmd="q" title="Turn Left">
              <span class="ph ph-caret-left"></span>
            </button>
            <button class="dpad-btn dpad-center" data-cmd="s" title="Stop">
              <span class="ph ph-stop"></span>
            </button>
            <button class="dpad-btn dpad-right" data-cmd="e" title="Turn Right">
              <span class="ph ph-caret-right"></span>
            </button>
            <button class="dpad-btn dpad-down" data-cmd="b" title="Backward">
              <span class="ph ph-caret-down"></span>
            </button>
          </div>
        </div>
      </div>

      <div class="card">
        <div class="card-header">
          <h3>Speed</h3>
        </div>
        <div class="card-body">
          <div class="speed-buttons">
            <button class="btn btn--secondary speed-btn" data-speed="5">50%</button>
            <button class="btn btn--secondary speed-btn" data-speed="6">60%</button>
            <button class="btn btn--secondary speed-btn" data-speed="7">70%</button>
            <button class="btn btn--primary speed-btn" data-speed="8">80%</button>
            <button class="btn btn--secondary speed-btn" data-speed="9">90%</button>
            <button class="btn btn--secondary speed-btn" data-speed="0">100%</button>
          </div>
        </div>
      </div>
    </div>

    <div class="card" style="margin-top: 1rem;">
      <div class="card-header">
        <h3>Keyboard Shortcuts</h3>
      </div>
      <div class="card-body">
        <table class="data-table">
          <thead><tr><th>Key</th><th>Action</th></tr></thead>
          <tbody>
            <tr><td><kbd>W</kbd> / <kbd>&uarr;</kbd></td><td>Forward</td></tr>
            <tr><td><kbd>S</kbd> / <kbd>&darr;</kbd></td><td>Backward</td></tr>
            <tr><td><kbd>A</kbd> / <kbd>&larr;</kbd></td><td>Turn Left</td></tr>
            <tr><td><kbd>D</kbd> / <kbd>&rarr;</kbd></td><td>Turn Right</td></tr>
            <tr><td><kbd>Space</kbd></td><td>Stop</td></tr>
          </tbody>
        </table>
      </div>
    </div>
  `;

  // D-Pad button clicks
  container.querySelectorAll('.dpad-btn').forEach((btn) => {
    btn.addEventListener('click', () => {
      const cmd = (btn as HTMLElement).dataset.cmd;
      if (cmd) sendMovementCommand(cmd);
    });
  });

  // Speed button clicks
  container.querySelectorAll('.speed-btn').forEach((btn) => {
    btn.addEventListener('click', () => {
      const speed = (btn as HTMLElement).dataset.speed;
      if (speed) sendMovementCommand(speed);
    });
  });

  // Keyboard controls
  const keyHandler = (e: KeyboardEvent) => {
    if (!container.classList.contains('active')) return;
    const keyMap: Record<string, string> = {
      w: 'f', ArrowUp: 'f',
      s: 'b', ArrowDown: 'b',
      a: 'q', ArrowLeft: 'q',
      d: 'e', ArrowRight: 'e',
      ' ': 's',
    };
    const cmd = keyMap[e.key];
    if (cmd) {
      e.preventDefault();
      sendMovementCommand(cmd);
    }
  };
  document.addEventListener('keydown', keyHandler);
}

export function destroyMovement() {
  // cleanup handled by component lifecycle
}
