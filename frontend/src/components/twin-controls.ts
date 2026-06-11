// Control panel — joint sliders, gripper toggle, mode selector, emergency stop

import { getTwinState, updateTwinState } from '../state/twin-state';
import { sendMovementCommand, fetchPaths } from '../api';
import { loadWaypointPath, clearWaypointPath, startSimulation, stopSimulation, getSimulationMode, getSimulationPresets, commandSimulation, stopSimulationRobot, startRecording, stopRecording, isRecording, getRecordingFrameCount, loadPlayback, startPlayback, pausePlayback, resumePlayback, stopPlayback, setPlaybackSpeed, getPlaybackProgress, getPlaybackState, seekPlaybackPercent } from './digital-twin';
import { loadAllRecordings, type Recording } from '../engine/recording';

let containerEl: HTMLElement | null = null;
let activeMovementCmd: string | null = null;

export function initTwinControls(parent: HTMLElement) {
  if (containerEl) return;

  containerEl = document.createElement('div');
  containerEl.className = 'twin-controls';
  containerEl.innerHTML = `
    <div class="twin-control-section">
      <h4>Gripper</h4>
      <button class="twin-btn" id="gripper-toggle">Open / Close</button>
    </div>
    <div class="twin-control-section">
      <h4>Camera Tilt</h4>
      <input type="range" id="tilt-slider" min="0" max="145" value="45" class="twin-slider" />
      <span class="twin-slider-value" id="tilt-value">45°</span>
    </div>
    <div class="twin-control-section">
      <h4>Lifter</h4>
      <div class="lifter-labels"><span>DOWN</span><span>UP</span></div>
      <input type="range" id="lifter-slider" min="0" max="100" value="50" class="twin-slider" />
      <span class="twin-slider-value" id="lifter-value">50mm</span>
      <div class="lifter-limits">
        <span class="limit-indicator" id="lifter-bottom-limit">▼ BOT</span>
        <span class="limit-indicator" id="lifter-top-limit">▲ TOP</span>
      </div>
    </div>
    <div class="twin-control-section">
      <h4>Waypoint Path</h4>
      <select id="path-selector" class="input input--sm" style="width:100%;margin-bottom:8px">
        <option value="">No path loaded</option>
      </select>
      <button class="twin-btn" id="clear-path" style="width:100%">Clear Path</button>
    </div>
    <div class="twin-control-section">
      <h4>Simulation</h4>
      <select id="sim-preset" class="input input--sm" style="width:100%;margin-bottom:8px">
        <option value="">Select scenario...</option>
      </select>
      <div style="display:flex;gap:4px;margin-bottom:8px">
        <button class="twin-btn" id="sim-start" style="flex:1">Start</button>
        <button class="twin-btn" id="sim-stop" style="flex:1">Stop</button>
      </div>
      <div id="sim-status" style="font-size:11px;color:var(--cds-text-placeholder);text-align:center">
        Mode: <span id="sim-mode-label">REAL</span>
      </div>
    </div>
    <div class="twin-control-section">
      <h4>Recording</h4>
      <div style="display:flex;gap:4px;margin-bottom:8px">
        <button class="twin-btn" id="rec-start" style="flex:1">Record</button>
        <button class="twin-btn" id="rec-stop" style="flex:1" disabled>Stop</button>
      </div>
      <div id="rec-status" style="font-size:11px;color:var(--cds-text-placeholder);text-align:center">
        Frames: <span id="rec-frames">0</span>
      </div>
    </div>
    <div class="twin-control-section">
      <h4>Playback</h4>
      <select id="playback-selector" class="input input--sm" style="width:100%;margin-bottom:8px">
        <option value="">No recording loaded</option>
      </select>
      <div style="display:flex;gap:4px;margin-bottom:6px">
        <button class="twin-btn" id="play-play" style="flex:1">Play</button>
        <button class="twin-btn" id="play-pause" style="flex:1" disabled>Pause</button>
        <button class="twin-btn" id="play-stop" style="flex:1" disabled>Stop</button>
      </div>
      <input type="range" id="playback-scrub" min="0" max="1000" value="0" class="twin-slider" style="width:100%;margin-bottom:4px" />
      <div style="display:flex;gap:4px;align-items:center">
        <span style="font-size:10px;color:var(--cds-text-placeholder)">Speed:</span>
        <button class="twin-btn play-speed" data-speed="0.25" style="padding:2px 6px;font-size:10px">0.25x</button>
        <button class="twin-btn play-speed" data-speed="0.5" style="padding:2px 6px;font-size:10px">0.5x</button>
        <button class="twin-btn play-speed" data-speed="1" style="padding:2px 6px;font-size:10px">1x</button>
        <button class="twin-btn play-speed" data-speed="2" style="padding:2px 6px;font-size:10px">2x</button>
        <button class="twin-btn play-speed" data-speed="4" style="padding:2px 6px;font-size:10px">4x</button>
      </div>
    </div>
    <div class="twin-control-section">
      <h4>Movement</h4>
      <div class="twin-movement-grid">
        <button class="twin-btn move-btn" data-cmd="f">Forward</button>
        <button class="twin-btn move-btn" data-cmd="b">Back</button>
        <button class="twin-btn move-btn" data-cmd="q">Forward-L</button>
        <button class="twin-btn move-btn" data-cmd="e">Forward-R</button>
        <button class="twin-btn move-btn" data-cmd="z">Back-L</button>
        <button class="twin-btn move-btn" data-cmd="x">Back-R</button>
        <button class="twin-btn move-btn" data-cmd="t">Turn L</button>
        <button class="twin-btn move-btn" data-cmd="y">Turn R</button>
        <button class="twin-btn move-btn stop-btn" data-cmd="s">Stop</button>
      </div>
    </div>
    <div class="twin-control-section">
      <button class="twin-btn estop-btn" id="twin-estop">EMERGENCY STOP</button>
    </div>
  `;
  parent.appendChild(containerEl);

  // Gripper toggle
  const gripperBtn = containerEl.querySelector('#gripper-toggle') as HTMLButtonElement;
  gripperBtn.addEventListener('click', () => {
    const open = !getTwinState().gripperOpen;
    updateTwinState({ gripperOpen: open });
    gripperBtn.textContent = open ? 'Open / Close' : 'Open / Close';
    sendMovementCommand(open ? 'go' : 'gc').catch(() => {});
  });

  // Tilt slider
  const tiltSlider = containerEl.querySelector('#tilt-slider') as HTMLInputElement;
  const tiltValue = containerEl.querySelector('#tilt-value') as HTMLElement;
  tiltSlider.addEventListener('input', () => {
    const val = parseInt(tiltSlider.value);
    tiltValue.textContent = val + '°';
    updateTwinState({ tiltAngle: val });
    sendMovementCommand(`ta${val}`).catch(() => {});
  });

  // Lifter slider with limit switch indicators
  const lifterSlider = containerEl.querySelector('#lifter-slider') as HTMLInputElement;
  const lifterValue = containerEl.querySelector('#lifter-value') as HTMLElement;
  const lifterBotLimit = containerEl.querySelector('#lifter-bottom-limit') as HTMLElement;
  const lifterTopLimit = containerEl.querySelector('#lifter-top-limit') as HTMLElement;

  function updateLifterLimits(val: number) {
    lifterBotLimit.classList.toggle('active', val <= 1);
    lifterTopLimit.classList.toggle('active', val >= 99);
  }

  lifterSlider.addEventListener('input', () => {
    const val = parseInt(lifterSlider.value);
    lifterValue.textContent = val + 'mm';
    updateLifterLimits(val);
    updateTwinState({ lifterHeight: val });
    sendMovementCommand(`lu${val}`).catch(() => {});
  });
  updateLifterLimits(parseInt(lifterSlider.value));

  // Path selector
  const pathSelector = containerEl.querySelector('#path-selector') as HTMLSelectElement;
  const clearPathBtn = containerEl.querySelector('#clear-path') as HTMLButtonElement;

  // Load available paths
  fetchPaths()
    .then((data) => {
      if (data.paths) {
        data.paths.forEach((path) => {
          const option = document.createElement('option');
          option.value = String(path.id);
          option.textContent = `${path.name} (${path.waypoint_count} pts)`;
          pathSelector.appendChild(option);
        });
      }
    })
    .catch(() => {});

  pathSelector.addEventListener('change', () => {
    const pathId = parseInt(pathSelector.value);
    if (!isNaN(pathId)) {
      loadWaypointPath(pathId);
    }
  });

  clearPathBtn.addEventListener('click', () => {
    pathSelector.value = '';
    clearWaypointPath();
  });

  // Simulation controls
  const simPreset = containerEl.querySelector('#sim-preset') as HTMLSelectElement;
  const simStartBtn = containerEl.querySelector('#sim-start') as HTMLButtonElement;
  const simStopBtn = containerEl.querySelector('#sim-stop') as HTMLButtonElement;
  const simModeLabel = containerEl.querySelector('#sim-mode-label') as HTMLElement;

  // Load simulation presets
  const presets = getSimulationPresets();
  presets.forEach((p) => {
    const option = document.createElement('option');
    option.value = p.name;
    option.textContent = `${p.name} — ${p.description}`;
    simPreset.appendChild(option);
  });

  simStartBtn.addEventListener('click', () => {
    const presetName = simPreset.value || undefined;
    startSimulation(presetName);
    simModeLabel.textContent = 'SIMULATION';
    simModeLabel.style.color = 'var(--cds-support-warning)';
  });

  simStopBtn.addEventListener('click', () => {
    stopSimulation();
    simModeLabel.textContent = 'REAL';
    simModeLabel.style.color = 'var(--cds-text-placeholder)';
  });

  // Recording controls
  const recStartBtn = containerEl.querySelector('#rec-start') as HTMLButtonElement;
  const recStopBtn = containerEl.querySelector('#rec-stop') as HTMLButtonElement;
  const recFrames = containerEl.querySelector('#rec-frames') as HTMLElement;

  recStartBtn.addEventListener('click', () => {
    startRecording();
    recStartBtn.disabled = true;
    recStopBtn.disabled = false;
    recFrames.textContent = '0';
  });

  recStopBtn.addEventListener('click', () => {
    const recording = stopRecording();
    recStartBtn.disabled = false;
    recStopBtn.disabled = true;
    if (recording) {
      refreshPlaybackSelector();
    }
  });

  // Update recording frame count periodically
  setInterval(() => {
    if (isRecording()) {
      recFrames.textContent = String(getRecordingFrameCount());
    }
  }, 500);

  // Playback controls
  const playbackSelector = containerEl.querySelector('#playback-selector') as HTMLSelectElement;
  const playPlayBtn = containerEl.querySelector('#play-play') as HTMLButtonElement;
  const playPauseBtn = containerEl.querySelector('#play-pause') as HTMLButtonElement;
  const playStopBtn = containerEl.querySelector('#play-stop') as HTMLButtonElement;
  const playbackScrub = containerEl.querySelector('#playback-scrub') as HTMLInputElement;

  function refreshPlaybackSelector() {
    const recordings = loadAllRecordings();
    playbackSelector.innerHTML = '<option value="">No recording loaded</option>';
    recordings.forEach((rec) => {
      const option = document.createElement('option');
      option.value = rec.name;
      const dur = (rec.duration / 1000).toFixed(1);
      option.textContent = `${rec.name} (${rec.frames.length} frames, ${dur}s)`;
      playbackSelector.appendChild(option);
    });
  }

  refreshPlaybackSelector();

  playbackSelector.addEventListener('change', () => {
    const name = playbackSelector.value;
    if (!name) return;
    const recordings = loadAllRecordings();
    const rec = recordings.find((r) => r.name === name);
    if (rec) {
      loadPlayback(rec);
      playPlayBtn.disabled = false;
    }
  });

  playPlayBtn.addEventListener('click', () => {
    startPlayback(
      (frame, progress) => {
        // Update twin state from recorded frame
        updateTwinState({
          position: { x: frame.x, y: frame.y, z: 0, roll: 0, pitch: 0, yaw: frame.heading },
          heading: frame.heading,
        });
        playbackScrub.value = String(Math.round(progress.progress * 1000));
      },
      () => {
        playPlayBtn.disabled = false;
        playPauseBtn.disabled = true;
        playStopBtn.disabled = true;
      }
    );
    playPlayBtn.disabled = true;
    playPauseBtn.disabled = false;
    playStopBtn.disabled = false;
  });

  playPauseBtn.addEventListener('click', () => {
    const state = getPlaybackState();
    if (state === 'playing') {
      pausePlayback();
      playPauseBtn.textContent = 'Resume';
    } else {
      resumePlayback();
      playPauseBtn.textContent = 'Pause';
    }
  });

  playStopBtn.addEventListener('click', () => {
    stopPlayback();
    playPlayBtn.disabled = false;
    playPauseBtn.disabled = true;
    playStopBtn.disabled = true;
    playPauseBtn.textContent = 'Pause';
    playbackScrub.value = '0';
  });

  playbackScrub.addEventListener('input', () => {
    const pct = parseInt(playbackScrub.value) / 1000;
    const progress = getPlaybackProgress();
    if (progress) {
      seekPlaybackPercent(pct);
    }
  });

  // Speed buttons
  containerEl.querySelectorAll('.play-speed').forEach((btn) => {
    btn.addEventListener('click', () => {
      const speed = parseFloat((btn as HTMLElement).dataset.speed || '1');
      setPlaybackSpeed(speed);
    });
  });

  // Movement buttons — click-to-toggle (click once to start, click again or click Stop to stop)
  const SIM_CMD_MAP: Record<string, { vx: number; vy: number; omega: number }> = {
    f: { vx: 0, vy: 300, omega: 0 },
    b: { vx: 0, vy: -300, omega: 0 },
    q: { vx: -212, vy: 212, omega: 0 },
    e: { vx: 212, vy: 212, omega: 0 },
    z: { vx: -212, vy: -212, omega: 0 },
    x: { vx: 212, vy: -212, omega: 0 },
    t: { vx: 0, vy: 0, omega: 90 },
    y: { vx: 0, vy: 0, omega: -90 },
    s: { vx: 0, vy: 0, omega: 0 },
  };

  function stopActiveMovement() {
    if (!activeMovementCmd) return;
    activeMovementCmd = null;
    if (getSimulationMode() === 'simulation') {
      stopSimulationRobot();
    } else {
      sendMovementCommand('s').catch(() => {});
    }
    containerEl?.querySelectorAll('.move-btn').forEach((b) => b.classList.remove('active'));
  }

  function handleMovementClick(cmd: string) {
    if (cmd === 's') {
      stopActiveMovement();
      return;
    }

    // If same command clicked again, stop
    if (activeMovementCmd === cmd) {
      stopActiveMovement();
      return;
    }

    // Stop any previous movement
    stopActiveMovement();

    activeMovementCmd = cmd;

    // Highlight active button
    containerEl?.querySelectorAll('.move-btn').forEach((b) => b.classList.remove('active'));
    containerEl?.querySelector(`.move-btn[data-cmd="${cmd}"]`)?.classList.add('active');

    if (getSimulationMode() === 'simulation') {
      const vel = SIM_CMD_MAP[cmd];
      if (vel) commandSimulation(vel.vx, vel.vy, vel.omega);
    } else {
      sendMovementCommand(cmd).catch(() => {});
    }
  }

  let lastTouchTime = 0;

  containerEl.querySelectorAll('.move-btn').forEach((btn) => {
    const cmd = (btn as HTMLElement).dataset.cmd;
    if (!cmd) return;

    // Touch events — fire immediately, suppress subsequent click
    btn.addEventListener('touchstart', (e) => {
      e.preventDefault();
      lastTouchTime = Date.now();
      handleMovementClick(cmd);
    }, { passive: false });

    // Mouse click — skip if a touch just fired (<100ms ago)
    btn.addEventListener('click', (e) => {
      e.preventDefault();
      if (Date.now() - lastTouchTime < 100) return;
      handleMovementClick(cmd);
    });
  });

  // Emergency stop
  const estopBtn = containerEl.querySelector('#twin-estop') as HTMLButtonElement;
  estopBtn.addEventListener('click', async () => {
    try {
      await sendMovementCommand('x');
    } catch {}
  });

  // Sync mode label with actual state (initDigitalTwin auto-starts simulation)
  if (getSimulationMode() === 'simulation') {
    simModeLabel.textContent = 'SIMULATION';
    simModeLabel.style.color = 'var(--cds-support-warning)';
  }
}

export function destroyTwinControls() {
  if (containerEl) {
    containerEl.remove();
    containerEl = null;
  }
}

export function syncSlidersToState() {
  const state = getTwinState();
  const lifterSlider = containerEl?.querySelector('#lifter-slider') as HTMLInputElement | null;
  if (lifterSlider) lifterSlider.value = String(state.lifterHeight);
  const tiltSlider = containerEl?.querySelector('#tilt-slider') as HTMLInputElement | null;
  if (tiltSlider) tiltSlider.value = String(state.tiltAngle);
}
