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

// --- AI Decision Engine ---

export interface AiStatus {
  running: boolean;
  mode: 'replay' | 'ifttt' | 'ai';
  task_goal: string;
  loop_interval: number;
  backend: string;
  camera: {
    state: string;
    camera_id: number;
    resolution: string;
    frame_count: number;
    error_count: number;
    last_frame_time: number;
  };
  last_decision: AiDecisionResult | null;
  history_count: number;
  human_guidance: string | null;
}

export interface AiDecisionResult {
  reasoning: string;
  actions: Array<{ type: string; value: string; delay_ms: number }>;
  confidence: number;
  continue: boolean;
  mode: string;
  latency_ms: number;
  timestamp: string;
}

export interface AiDecision {
  id: number;
  taskGoal: string;
  mode: string;
  aiResponse: AiDecisionResult | null;
  actionsExecuted: number;
  confidence: number | null;
  backend: string;
  modelUsed: string | null;
  latencyMs: number | null;
  createdAt: string | null;
}

// --- Waypoint Memory ---

export interface SavedPath {
  id: number;
  name: string;
  description: string | null;
  waypoint_count: number;
  created_at: string | null;
}

export interface Waypoint {
  id: number;
  order: number;
  x: number;
  y: number;
  heading: number;
  actions: Record<string, unknown> | null;
  sensorSnapshot: Record<string, unknown> | null;
}

export interface WaypointStatus {
  recording: boolean;
  current_path_id: number | null;
  replaying: boolean;
  replay_index: number;
}

// --- Robot Position & Map ---

export interface RobotPosition {
  x: number;
  y: number;
  z: number;
  roll: number;
  pitch: number;
  yaw: number;
}

export interface MapWaypoint {
  x: number;
  y: number;
  heading: number;
  order: number;
}

export interface SensorReadings {
  laser_left_front: number;
  laser_left_back: number;
  laser_right_front: number;
  laser_right_back: number;
  laser_back_left: number;
  laser_back_right: number;
  ultra_front_left: number;
  ultra_front_right: number;
}

// --- Automation Logs ---

export interface AutomationLogEntry {
  id: number;
  automationId: number;
  automationName: string;
  triggerReason: string;
  actionsExecuted: number;
  timestamp: string;
}
