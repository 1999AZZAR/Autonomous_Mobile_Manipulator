// Movement control component — D-Pad + speed control
// Triangular omni wheel: FR(1), FL(2), Back(3)
// Movement: f/b (FR+FL), l/r (diagonal), t/y (rotate all 3)

import { sendMovementCommand } from '../api';

let keyHandler: ((e: KeyboardEvent) => void) | null = null;

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
            <button class="dpad-btn dpad-up" data-cmd="f" title="Forward (FR+FL)">
              <span class="ph ph-caret-up"></span>
            </button>
            <button class="dpad-btn dpad-left" data-cmd="t" title="Rotate Left (all 3)">
              <span class="ph ph-caret-left"></span>
            </button>
            <button class="dpad-btn dpad-center" data-cmd="s" title="Stop">
              <span class="ph ph-stop"></span>
            </button>
            <button class="dpad-btn dpad-right" data-cmd="y" title="Rotate Right (all 3)">
              <span class="ph ph-caret-right"></span>
            </button>
            <button class="dpad-btn dpad-down" data-cmd="b" title="Backward (FR+FL)">
              <span class="ph ph-caret-down"></span>
            </button>
          </div>
          <div class="dpad-diag" style="display:flex;justify-content:center;gap:0.5rem;margin-top:0.5rem;">
            <button class="btn btn--secondary" data-cmd="q" title="Forward-Left (FR+Back)">↖ FL</button>
            <button class="btn btn--secondary" data-cmd="e" title="Forward-Right (FL+Back)">↗ FR</button>
            <button class="btn btn--secondary" data-cmd="z" title="Backward-Left (Back-FR)">↙ BL</button>
            <button class="btn btn--secondary" data-cmd="x" title="Backward-Right (Back-FL)">↘ BR</button>
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
          <thead><tr><th>Key</th><th>Action</th><th>Wheels</th></tr></thead>
          <tbody>
            <tr><td><kbd>W</kbd> / <kbd>&uarr;</kbd></td><td>Forward</td><td>FR + FL</td></tr>
            <tr><td><kbd>S</kbd> / <kbd>&darr;</kbd></td><td>Backward</td><td>FR + FL</td></tr>
            <tr><td><kbd>A</kbd> / <kbd>&larr;</kbd></td><td>Rotate Left</td><td>All 3</td></tr>
            <tr><td><kbd>D</kbd> / <kbd>&rarr;</kbd></td><td>Rotate Right</td><td>All 3</td></tr>
            <tr><td><kbd>Q</kbd></td><td>Forward-Left</td><td>FR + Back</td></tr>
            <tr><td><kbd>E</kbd></td><td>Forward-Right</td><td>FL + Back</td></tr>
            <tr><td><kbd>Z</kbd></td><td>Backward-Left</td><td>Back - FR</td></tr>
            <tr><td><kbd>X</kbd></td><td>Backward-Right</td><td>Back - FL</td></tr>
            <tr><td><kbd>Space</kbd></td><td>Stop</td><td>All</td></tr>
          </tbody>
        </table>
      </div>
    </div>
  `;

  // D-Pad + diagonal button clicks
  container.querySelectorAll('.dpad-btn, .dpad-diag .btn').forEach((btn) => {
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
  keyHandler = (e: KeyboardEvent) => {
    if (!container.classList.contains('active')) return;
    const keyMap: Record<string, string> = {
      w: 'f', ArrowUp: 'f',
      s: 'b', ArrowDown: 'b',
      a: 't', ArrowLeft: 't',
      d: 'y', ArrowRight: 'y',
      q: 'q',
      e: 'e',
      z: 'z',
      x: 'x',
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
  if (keyHandler) {
    document.removeEventListener('keydown', keyHandler);
    keyHandler = null;
  }
}
