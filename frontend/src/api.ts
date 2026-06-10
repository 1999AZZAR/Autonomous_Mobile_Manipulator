// REST API client for the Python backend

import type { Automation, SensorData, SystemStatus, AiStatus, AiDecision, SavedPath, WaypointStatus } from './types';

const BASE = '/api';

async function request<T>(path: string, opts?: RequestInit): Promise<T> {
  const res = await fetch(`${BASE}${path}`, {
    headers: { 'Content-Type': 'application/json' },
    ...opts,
  });
  if (!res.ok) {
    const body = await res.json().catch(() => ({}));
    throw new Error(body.error || `HTTP ${res.status}`);
  }
  return res.json();
}

// --- Sensors ---

export function fetchSensors(): Promise<SensorData> {
  return request<SensorData>('/sensors');
}

// --- System ---

export function fetchSystemStatus(): Promise<SystemStatus> {
  return request<SystemStatus>('/status');
}

// --- Robot control ---

export function sendMovementCommand(command: string): Promise<unknown> {
  return request('/command', {
    method: 'POST',
    body: JSON.stringify({ command }),
  });
}

// --- Automations ---

export function fetchAutomations(): Promise<{ automations: Automation[] }> {
  return request('/automations');
}

export function fetchAutomation(id: string): Promise<{ automation: Automation }> {
  return request(`/automations/${id}`);
}

export function createAutomation(data: {
  name: string;
  description?: string;
  triggerType: string;
  conditions: Omit<Automation['conditions'][0], 'id' | 'automationId'>[];
  actions: Omit<Automation['actions'][0], 'id' | 'automationId'>[];
}): Promise<{ id: string }> {
  return request('/automations', {
    method: 'POST',
    body: JSON.stringify(data),
  });
}

export function updateAutomation(
  id: string,
  data: Partial<Automation>
): Promise<unknown> {
  return request(`/automations/${id}`, {
    method: 'PUT',
    body: JSON.stringify(data),
  });
}

export function deleteAutomation(id: string): Promise<unknown> {
  return request(`/automations/${id}`, { method: 'DELETE' });
}

export function toggleAutomation(id: string): Promise<unknown> {
  return request(`/automations/${id}/toggle`, { method: 'POST' });
}

export function runAutomation(id: string): Promise<unknown> {
  return request(`/automations/${id}/run`, { method: 'POST' });
}

export function fetchAutomationLogs(id: string): Promise<{ logs: unknown[] }> {
  return request(`/automations/${id}/logs`);
}

// --- Feeds ---

export function fetchFeeds(): Promise<{ feeds: Record<string, unknown> }> {
  return request('/feeds');
}

export function fetchFeedValue(key: string): Promise<{ key: string; value: unknown }> {
  return request(`/feeds/${key}`);
}

// --- AI Decision Engine ---

export function fetchAiStatus(): Promise<AiStatus> {
  return request('/ai/status');
}

export function startAiLoop(goal: string, interval?: number): Promise<{ success: boolean }> {
  return request('/ai/start', {
    method: 'POST',
    body: JSON.stringify({ goal, interval }),
  });
}

export function stopAiLoop(): Promise<{ success: boolean }> {
  return request('/ai/stop', { method: 'POST' });
}

export function analyzeOnce(goal?: string): Promise<unknown> {
  return request('/ai/analyze', {
    method: 'POST',
    body: JSON.stringify({ goal }),
  });
}

export function fetchAiDecisions(limit = 20): Promise<{ decisions: AiDecision[] }> {
  return request(`/ai/decisions?limit=${limit}`);
}

export function sendHumanGuidance(guidance: string): Promise<{ success: boolean }> {
  return request('/ai/guidance', {
    method: 'POST',
    body: JSON.stringify({ guidance }),
  });
}

export function updateAiConfig(config: {
  interval?: number;
  backend?: string;
  model?: string;
  goal?: string;
}): Promise<{ success: boolean }> {
  return request('/ai/config', {
    method: 'POST',
    body: JSON.stringify(config),
  });
}

export function getCameraSnapshotUrl(): string {
  return `${BASE}/ai/camera/snapshot`;
}

// --- Waypoints ---

export function fetchPaths(): Promise<{ paths: SavedPath[] }> {
  return request('/waypoints/paths');
}

export function fetchPath(id: number): Promise<SavedPath & { waypoints: Array<{ id: number; order: number; x: number; y: number; heading: number; actions: unknown; sensorSnapshot: unknown }> }> {
  return request(`/waypoints/paths/${id}`);
}

export function createPath(name: string, description?: string): Promise<{ success: boolean; path_id: number }> {
  return request('/waypoints/paths', {
    method: 'POST',
    body: JSON.stringify({ name, description }),
  });
}

export function deletePath(id: number): Promise<{ success: boolean }> {
  return request(`/waypoints/paths/${id}`, { method: 'DELETE' });
}

export function recordWaypoint(actions?: Record<string, unknown>): Promise<{ success: boolean }> {
  return request('/waypoints/record', {
    method: 'POST',
    body: JSON.stringify({ actions }),
  });
}

export function stopRecording(): Promise<{ success: boolean; path_id: number; waypoint_count: number }> {
  return request('/waypoints/stop', { method: 'POST' });
}

export function startReplay(pathId: number): Promise<{ success: boolean }> {
  return request(`/waypoints/replay/${pathId}`, { method: 'POST' });
}

export function stopReplay(): Promise<{ success: boolean }> {
  return request('/waypoints/replay/stop', { method: 'POST' });
}

export function fetchWaypointStatus(): Promise<WaypointStatus> {
  return request('/waypoints/status');
}
