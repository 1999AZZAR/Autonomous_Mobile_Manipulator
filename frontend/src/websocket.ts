// SSE client for real-time sensor data

import type { SensorData } from './types';

type MessageHandler = (data: SensorData) => void;

export class SensorSocket {
  private es: EventSource | null = null;
  private handlers: MessageHandler[] = [];
  private reconnectTimer: ReturnType<typeof setTimeout> | null = null;
  private url: string;
  private destroyed = false;

  constructor() {
    this.url = `${location.origin}/ws/sensors`;
  }

  connect() {
    if (this.destroyed) return;
    this.disconnect();
    this.es = new EventSource(this.url);

    this.es.onmessage = (e) => {
      try {
        const data: SensorData = JSON.parse(e.data);
        this.handlers.forEach((h) => h(data));
      } catch { /* ignore malformed */ }
    };

    this.es.onerror = () => {
      this.updateStatus(false);
      this.es?.close();
      this.reconnectTimer = setTimeout(() => this.connect(), 2000);
    };

    this.es.onopen = () => {
      this.updateStatus(true);
    };
  }

  disconnect() {
    if (this.reconnectTimer) clearTimeout(this.reconnectTimer);
    this.es?.close();
    this.es = null;
    this.updateStatus(false);
  }

  destroy() {
    this.destroyed = true;
    this.disconnect();
  }

  onSensorData(handler: MessageHandler) {
    this.handlers.push(handler);
    return () => {
      this.handlers = this.handlers.filter((h) => h !== handler);
    };
  }

  private updateStatus(connected: boolean) {
    const el = document.getElementById('ws-status');
    if (!el) return;
    const dot = el.querySelector('.status-dot');
    const text = el.querySelector('.status-text');
    if (dot) {
      dot.className = `status-dot ${connected ? 'connected' : 'disconnected'}`;
    }
    if (text) {
      text.textContent = connected ? 'Connected' : 'Disconnected';
    }
  }
}
