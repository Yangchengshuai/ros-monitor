import { create } from 'zustand';
import type { ConnectionInfo } from '../types/api';

interface SystemStore {
  // 连接状态
  connection: ConnectionInfo;
  
  // 性能指标
  performance: {
    wsLatency: number;
    dataRate: number;
    errorCount: number;
    uptime: number;
  };
  
  // UI状态
  ui: {
    sidebarCollapsed: boolean;
    theme: 'light' | 'dark';
    currentPage: string;
  };
  
  // Actions
  updateConnectionStatus: (type: keyof ConnectionInfo, status: boolean) => void;
  updateLatency: (latency: number) => void;
  updatePerformanceMetrics: (metrics: Partial<SystemStore['performance']>) => void;
  toggleSidebar: () => void;
  setCurrentPage: (page: string) => void;
  incrementErrorCount: () => void;
  resetErrorCount: () => void;
}

export const useSystemStore = create<SystemStore>((set) => ({
  connection: {
    websocket: false,
    api: false,
    ros: false,
    latency: 0,
  },
  
  performance: {
    wsLatency: 0,
    dataRate: 0,
    errorCount: 0,
    uptime: 0,
  },
  
  ui: {
    sidebarCollapsed: false,
    theme: 'dark',
    currentPage: 'dashboard',
  },
  
  updateConnectionStatus: (type: keyof ConnectionInfo, status: boolean) => set((state) => ({
    connection: {
      ...state.connection,
      [type]: status,
    },
  })),
  
  updateLatency: (latency: number) => set((state) => ({
    connection: {
      ...state.connection,
      latency,
    },
    performance: {
      ...state.performance,
      wsLatency: latency,
    },
  })),
  
  updatePerformanceMetrics: (metrics: Partial<SystemStore['performance']>) => set((state) => ({
    performance: {
      ...state.performance,
      ...metrics,
    },
  })),
  
  toggleSidebar: () => set((state) => ({
    ui: {
      ...state.ui,
      sidebarCollapsed: !state.ui.sidebarCollapsed,
    },
  })),
  
  setCurrentPage: (page: string) => set((state) => ({
    ui: {
      ...state.ui,
      currentPage: page,
    },
  })),
  
  incrementErrorCount: () => set((state) => ({
    performance: {
      ...state.performance,
      errorCount: state.performance.errorCount + 1,
    },
  })),
  
  resetErrorCount: () => set((state) => ({
    performance: {
      ...state.performance,
      errorCount: 0,
    },
  })),
}));

