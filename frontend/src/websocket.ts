// WebSocket client for real-time sensor data

import type { SensorData } from './types';

type MessageHandler = (data: SensorData) => void;

export class SensorSocket {
  private ws: WebSocket | null = null;
  private handlers: MessageHandler[] = [];
  private reconnectTimer: ReturnType<typeof setTimeout> | null = null;
  private url: string;

  constructor() {
    const proto = location.protocol === 'https:' ? 'wss:' : 'ws:';
    this.url = `${proto}//${location.host}/ws/sensors`;
  }

  connect() {
    if (this.ws?.readyState === WebSocket.OPEN) return;

    this.ws = new WebSocket(this.url);

    this.ws.onmessage = (e) => {
      try {
        const data: SensorData = JSON.parse(e.data);
        this.handlers.forEach((h) => h(data));
      } catch { /* ignore malformed */ }
    };

    this.ws.onclose = () => {
      this.updateStatus(false);
      this.reconnectTimer = setTimeout(() => this.connect(), 2000);
    };

    this.ws.onerror = () => {
      this.ws?.close();
    };

    this.ws.onopen = () => {
      this.updateStatus(true);
    };
  }

  disconnect() {
    if (this.reconnectTimer) clearTimeout(this.reconnectTimer);
    this.ws?.close();
    this.ws = null;
    this.updateStatus(false);
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
