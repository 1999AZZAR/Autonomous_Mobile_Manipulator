// Playback engine — load, play, pause, scrub, speed control

import type { Recording, RecordingFrame } from './recording';
import type { TwinState } from '../types/twin';

export type PlaybackState = 'idle' | 'playing' | 'paused';

export interface PlaybackProgress {
  state: PlaybackState;
  currentTime: number;    // ms
  totalTime: number;      // ms
  frameIndex: number;
  totalFrames: number;
  speed: number;          // multiplier
  progress: number;       // 0-1
}

export class PlaybackEngine {
  private recording: Recording | null = null;
  private playbackState: PlaybackState = 'idle';
  private frameIndex = 0;
  private elapsed = 0;
  private speed = 1;
  private lastTick = 0;
  private animFrame = 0;
  private onFrame: ((frame: RecordingFrame, progress: PlaybackProgress) => void) | null = null;
  private onComplete: (() => void) | null = null;

  load(recording: Recording) {
    this.stop();
    this.recording = recording;
    this.frameIndex = 0;
    this.elapsed = 0;
  }

  play(onFrame: (frame: RecordingFrame, progress: PlaybackProgress) => void, onComplete?: () => void) {
    if (!this.recording) return;
    this.onFrame = onFrame;
    this.onComplete = onComplete || null;
    this.playbackState = 'playing';
    this.lastTick = performance.now();
    this.tick();
  }

  pause() {
    this.playbackState = 'paused';
    cancelAnimationFrame(this.animFrame);
  }

  resume() {
    if (!this.recording) return;
    this.playbackState = 'playing';
    this.lastTick = performance.now();
    this.tick();
  }

  stop() {
    this.playbackState = 'idle';
    cancelAnimationFrame(this.animFrame);
    this.frameIndex = 0;
    this.elapsed = 0;
    this.onFrame = null;
    this.onComplete = null;
  }

  seek(timeMs: number) {
    if (!this.recording) return;
    // Find closest frame
    const idx = this.recording.frames.findIndex((f) => f.t >= timeMs);
    this.frameIndex = idx >= 0 ? idx : this.recording.frames.length - 1;
    this.elapsed = timeMs;
  }

  seekPercent(pct: number) {
    if (!this.recording) return;
    this.seek(pct * this.recording.duration);
  }

  setSpeed(speed: number) {
    this.speed = Math.max(0.25, Math.min(4, speed));
  }

  getSpeed(): number {
    return this.speed;
  }

  getProgress(): PlaybackProgress {
    if (!this.recording) {
      return { state: 'idle', currentTime: 0, totalTime: 0, frameIndex: 0, totalFrames: 0, speed: this.speed, progress: 0 };
    }
    return {
      state: this.playbackState,
      currentTime: this.elapsed,
      totalTime: this.recording.duration,
      frameIndex: this.frameIndex,
      totalFrames: this.recording.frames.length,
      speed: this.speed,
      progress: this.recording.duration > 0 ? this.elapsed / this.recording.duration : 0,
    };
  }

  getState(): PlaybackState {
    return this.playbackState;
  }

  private tick = () => {
    if (this.playbackState !== 'playing' || !this.recording) return;

    const now = performance.now();
    const dt = (now - this.lastTick) * this.speed;
    this.lastTick = now;
    this.elapsed += dt;

    const frames = this.recording.frames;

    // Advance frame index
    while (this.frameIndex < frames.length - 1 && frames[this.frameIndex].t < this.elapsed) {
      this.frameIndex++;
    }

    if (this.frameIndex >= frames.length - 1) {
      // Playback complete
      this.playbackState = 'idle';
      const lastFrame = frames[frames.length - 1];
      this.onFrame?.(lastFrame, this.getProgress());
      this.onComplete?.();
      return;
    }

    const frame = frames[this.frameIndex];
    this.onFrame?.(frame, this.getProgress());

    this.animFrame = requestAnimationFrame(this.tick);
  };
}

// Analysis utilities
export interface PlaybackAnalysis {
  totalDistance: number;      // mm
  totalTime: number;         // ms
  avgSpeed: number;          // mm/s
  maxSpeed: number;          // mm/s
  turns: number;
  sensorMinima: Record<string, number>;
}

export function analyzeRecording(recording: Recording): PlaybackAnalysis {
  const frames = recording.frames;
  if (frames.length < 2) {
    return { totalDistance: 0, totalTime: 0, avgSpeed: 0, maxSpeed: 0, turns: 0, sensorMinima: {} };
  }

  let totalDist = 0;
  let maxSpeed = 0;
  let turns = 0;
  const sensorMinima: Record<string, number> = {};

  for (let i = 1; i < frames.length; i++) {
    const dx = frames[i].x - frames[i - 1].x;
    const dy = frames[i].y - frames[i - 1].y;
    totalDist += Math.sqrt(dx * dx + dy * dy);

    const dt = (frames[i].t - frames[i - 1].t) / 1000;
    if (dt > 0) {
      const speed = Math.sqrt(dx * dx + dy * dy) / dt;
      maxSpeed = Math.max(maxSpeed, speed);
    }

    const dHeading = Math.abs(frames[i].heading - frames[i - 1].heading);
    if (dHeading > 10 && dHeading < 350) turns++;

    // Track sensor minima
    Object.keys(frames[i].sensors).forEach((key) => {
      const val = (frames[i].sensors as unknown as Record<string, number>)[key];
      if (typeof val === 'number') {
        if (!(key in sensorMinima) || val < sensorMinima[key]) {
          sensorMinima[key] = val;
        }
      }
    });
  }

  const duration = frames[frames.length - 1].t - frames[0].t;

  return {
    totalDistance: Math.round(totalDist),
    totalTime: duration,
    avgSpeed: duration > 0 ? Math.round(totalDist / (duration / 1000)) : 0,
    maxSpeed: Math.round(maxSpeed),
    turns,
    sensorMinima,
  };
}
