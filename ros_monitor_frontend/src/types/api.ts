// API接口类型定义

export interface ApiResponse<T = any> {
  success: boolean;
  message: string;
  data?: T;
  error?: string;
  timestamp?: string;
}

export interface HealthResponse {
  success: boolean;
  message: string;
  ros_ready: boolean;
}

export interface SystemStatus {
  ros_master: {
    connected: boolean;
    uri: string;
    topics: number;
    nodes: number;
  };
  sensors: {
    camera: SensorStatus;
    lidar: SensorStatus;
    imu: SensorStatus;
  };
  resources: {
    cpu_usage: number;
    memory_usage: number;
    disk_usage: number;
  };
  uptime: number;
}

export interface SensorStatus {
  active: boolean;
  frequency: number;
  last_message: string;
  error_count: number;
}

export interface ConnectionInfo {
  websocket: boolean;
  api: boolean;
  ros: boolean;
  latency: number;
}

