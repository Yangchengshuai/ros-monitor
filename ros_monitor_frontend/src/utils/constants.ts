// 应用常量配置

export const config = {
  // API配置
  API_BASE_URL: import.meta.env.VITE_API_BASE_URL || 'http://localhost:8001',
  WS_URL: import.meta.env.VITE_WS_URL || 'ws://localhost:8001',
  
  // 应用配置
  APP_TITLE: import.meta.env.VITE_APP_TITLE || 'ROS远程监控系统',
  
  // WebSocket配置
  WS_RECONNECT_ATTEMPTS: 5,
  WS_RECONNECT_INTERVAL: 3000,
  
  // 数据配置
  MAX_HISTORY_POINTS: 1000,
  CHART_UPDATE_INTERVAL: 100,
  HEALTH_CHECK_INTERVAL: 5000,
  
  // UI配置
  SIDEBAR_WIDTH: 240,
  HEADER_HEIGHT: 64,
};

export const SENSOR_TOPICS = {
  IMU: 'imu',
  CAMERA: 'camera',
  LIDAR: 'lidar',
} as const;

export const CONNECTION_STATUS = {
  CONNECTED: 'connected',
  DISCONNECTED: 'disconnected',
  CONNECTING: 'connecting',
  ERROR: 'error',
} as const;

export const CHART_COLORS = {
  X_AXIS: '#ff4d4f',
  Y_AXIS: '#52c41a', 
  Z_AXIS: '#1890ff',
  W_AXIS: '#722ed1',
} as const;

