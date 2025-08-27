import { create } from 'zustand';
import type { CameraData, LidarData, SensorStatus } from '../types/sensors';

interface SensorStore {
  // 相机数据
  camera: {
    left: CameraData | null;
    right: CameraData | null;
    status: SensorStatus;
  };
  
  // 激光雷达数据
  lidar: {
    latest: LidarData | null;
    status: SensorStatus;
  };
  
  // Actions
  updateCameraData: (camera: 'left' | 'right', data: CameraData) => void;
  updateLidarData: (data: LidarData) => void;
  updateSensorStatus: (sensor: 'camera' | 'lidar', status: Partial<SensorStatus>) => void;
  clearAllData: () => void;
}

const initialSensorStatus: SensorStatus = {
  connected: false,
  lastUpdate: 0,
  frequency: 0,
  errorCount: 0,
};

export const useSensorStore = create<SensorStore>((set) => ({
  camera: {
    left: null,
    right: null,
    status: { ...initialSensorStatus },
  },
  
  lidar: {
    latest: null,
    status: { ...initialSensorStatus },
  },
  
  updateCameraData: (camera: 'left' | 'right', data: CameraData) => set((state) => ({
    camera: {
      ...state.camera,
      [camera]: data,
      status: {
        connected: true,
        lastUpdate: Date.now(),
        frequency: state.camera.status.frequency,
        errorCount: state.camera.status.errorCount,
      },
    },
  })),
  
  updateLidarData: (data: LidarData) => set((state) => ({
    lidar: {
      latest: data,
      status: {
        connected: true,
        lastUpdate: Date.now(),
        frequency: state.lidar.status.frequency,
        errorCount: state.lidar.status.errorCount,
      },
    },
  })),
  
  updateSensorStatus: (sensor: 'camera' | 'lidar', status: Partial<SensorStatus>) => set((state) => {
    if (sensor === 'camera') {
      return {
        camera: {
          ...state.camera,
          status: {
            ...state.camera.status,
            ...status,
          },
        },
      };
    } else if (sensor === 'lidar') {
      return {
        lidar: {
          ...state.lidar,
          status: {
            ...state.lidar.status,
            ...status,
          },
        },
      };
    }
    return state;
  }),
  
  clearAllData: () => set(() => ({
    camera: {
      left: null,
      right: null,
      status: { ...initialSensorStatus },
    },
    lidar: {
      latest: null,
      status: { ...initialSensorStatus },
    },
  })),
}));

