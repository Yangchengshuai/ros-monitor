// 传感器数据类型定义

export interface CameraData {
  camera_id: string;
  timestamp: number;
  sequence: number;
  encoding: string;
  width: number;
  height: number;
  data: string; // Base64编码的图像数据
  compressed: boolean;
}

export interface LidarData {
  timestamp: number;
  frame_id: string;
  point_count: number;
  points: number[]; // 点云数据数组
}

export interface SensorStatus {
  connected: boolean;
  lastUpdate: number;
  frequency: number;
  errorCount: number;
}

