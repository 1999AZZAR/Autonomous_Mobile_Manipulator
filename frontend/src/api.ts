// REST API client for the Python backend

import type { Automation, SensorData, SystemStatus } from './types';

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
