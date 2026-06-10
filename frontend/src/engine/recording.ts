// Session recording — capture all state changes to JSON

import type { TwinState } from '../types/twin';
import type { MockSensorData } from './mock-sensors';

export interface RecordingFrame {
  t: number;          // timestamp ms
  x: number;          // mm
  y: number;          // mm
  heading: number;    // deg
  sensors: MockSensorData;
  gripperOpen: boolean;
  tiltAngle: number;
  lifterHeight: number;
  aiMode: string;
}

export interface Recording {
  name: string;
  startedAt: string;
  frames: RecordingFrame[];
  duration: number;   // ms
  sampleRate: number; // Hz
}

export class SessionRecorder {
  private frames: RecordingFrame[] = [];
  private startTime = 0;
  private recording = false;
  private sampleInterval: ReturnType<typeof setInterval> | null = null;
  private sampleRate: number;

  constructor(sampleRate: number = 10) {
    this.sampleRate = sampleRate;
  }

  start(name: string = `rec_${Date.now()}`): string {
    if (this.recording) return name;
    this.frames = [];
    this.startTime = performance.now();
    this.recording = true;

    const intervalMs = 1000 / this.sampleRate;
    this.sampleInterval = setInterval(() => {
      // Frame capture handled by external caller via addFrame()
    }, intervalMs);

    return name;
  }

  stop(): Recording | null {
    if (!this.recording) return null;
    this.recording = false;

    if (this.sampleInterval) {
      clearInterval(this.sampleInterval);
      this.sampleInterval = null;
    }

    const duration = performance.now() - this.startTime;

    return {
      name: `rec_${Date.now()}`,
      startedAt: new Date(this.startTime).toISOString(),
      frames: this.frames,
      duration,
      sampleRate: this.sampleRate,
    };
  }

  addFrame(state: TwinState, sensors: MockSensorData) {
    if (!this.recording) return;

    this.frames.push({
      t: Math.round(performance.now() - this.startTime),
      x: state.position.x,
      y: state.position.y,
      heading: state.heading,
      sensors,
      gripperOpen: state.gripperOpen,
      tiltAngle: state.tiltAngle,
      lifterHeight: state.lifterHeight,
      aiMode: state.mode,
    });
  }

  isRecording(): boolean {
    return this.recording;
  }

  getFrameCount(): number {
    return this.frames.length;
  }
}

// Local storage
const STORAGE_KEY = 'amm-twin-recordings';

export function saveRecording(recording: Recording): void {
  const all = loadAllRecordings();
  all.push(recording);
  // Keep max 20 recordings
  while (all.length > 20) all.shift();
  localStorage.setItem(STORAGE_KEY, JSON.stringify(all));
}

export function loadAllRecordings(): Recording[] {
  try {
    const data = localStorage.getItem(STORAGE_KEY);
    return data ? JSON.parse(data) : [];
  } catch {
    return [];
  }
}

export function deleteRecording(name: string): void {
  const all = loadAllRecordings().filter((r) => r.name !== name);
  localStorage.setItem(STORAGE_KEY, JSON.stringify(all));
}

export function exportRecording(recording: Recording): string {
  return JSON.stringify(recording, null, 2);
}

export function importRecording(json: string): Recording | null {
  try {
    const data = JSON.parse(json);
    if (data.frames && Array.isArray(data.frames)) {
      return data as Recording;
    }
    return null;
  } catch {
    return null;
  }
}
