// --- Types ---

export interface SensorData {
  laser_left_front: number;
  laser_left_back: number;
  laser_right_front: number;
  laser_right_back: number;
  laser_back_left: number;
  laser_back_right: number;
  ultra_front_left: number;
  ultra_front_right: number;
  line_left: number;
  line_center: number;
  line_right: number;
  imu_heading: number;
  imu_pitch: number;
  imu_roll: number;
  tf_luna_distance: number;
  mega_connected: boolean;
}

export interface SystemStatus {
  mega_connected: boolean;
  ros2_available: boolean;
  simulation_mode: boolean;
  uptime: number;
}

export interface Automation {
  id: string;
  name: string;
  description: string;
  enabled: boolean;
  triggerType: 'sensor' | 'time' | 'manual' | 'webhook';
  conditions: AutomationCondition[];
  actions: AutomationAction[];
  createdAt: string;
  updatedAt: string;
}

export interface AutomationCondition {
  id: string;
  automationId: string;
  feedKey: string;
  operator: '>' | '<' | '==' | '!=' | '>=' | '<=';
  threshold: number | string;
  logicGate: 'AND' | 'OR';
  conditionOrder: number;
}

export interface AutomationAction {
  id: string;
  automationId: string;
  actionType: string;
  parameters: Record<string, unknown>;
  actionOrder: number;
  delayMs: number;
}

export interface AutomationLog {
  id: string;
  automationId: string;
  triggeredAt: string;
  conditionsMet: boolean;
  actionsExecuted: number;
  result: string;
  sensorSnapshot: Record<string, unknown>;
}

export interface FeedValue {
  key: string;
  value: unknown;
}

export interface ApiResponse<T> {
  success?: boolean;
  error?: string;
  timestamp?: number;
  data?: T;
}
