import { useSystemStore } from '../stores/useSystemStore';
import { getApiBaseUrl } from '../utils/network';

export interface APIHealthResponse {
  success: boolean;
  message: string;
  ros_ready: boolean;
  timestamp: number;
  websocket_clients: number;
}

export interface SystemStatusResponse {
  success: boolean;
  message: string;
  data: {
    ros_connection: any;
    websocket_status: {
      total_clients: number;
      subscription_info: any;
    };
    timestamp: number;
  } | null;
}

class APIService {
  private baseURL: string = '';
  private healthCheckInterval: number | null = null;
  private isChecking = false;

  constructor() {
    // 动态获取当前主机地址和端口
    this.updateBaseURL();
  }

  /**
   * 动态更新API基础URL
   */
  private updateBaseURL(): void {
    this.baseURL = getApiBaseUrl();
    console.log('🔧 API基础URL已更新:', this.baseURL);
  }

  /**
   * 检查API健康状态
   */
  async checkHealth(): Promise<APIHealthResponse | null> {
    try {
      // 每次请求前更新地址（确保地址是最新的）
      this.updateBaseURL();
      
      const response = await fetch(`${this.baseURL}/api/v1/health`, {
        method: 'GET',
        headers: {
          'Content-Type': 'application/json',
        },
        signal: AbortSignal.timeout(5000), // 5秒超时
      });

      if (response.ok) {
        const data: APIHealthResponse = await response.json();
        return data;
      } else {
        console.error('API health check failed:', response.status, response.statusText);
        return null;
      }
    } catch (error) {
      console.error('API health check error:', error);
      return null;
    }
  }

  /**
   * 获取系统状态
   */
  async getSystemStatus(): Promise<SystemStatusResponse | null> {
    try {
      // 每次请求前更新地址
      this.updateBaseURL();
      
      const response = await fetch(`${this.baseURL}/api/v1/system/status`, {
        method: 'GET',
        headers: {
          'Content-Type': 'application/json',
        },
        signal: AbortSignal.timeout(5000), // 5秒超时
      });

      if (response.ok) {
        const data: SystemStatusResponse = await response.json();
        return data;
      } else {
        console.error('System status check failed:', response.status, response.statusText);
        return null;
      }
    } catch (error) {
      console.error('System status check error:', error);
      return null;
    }
  }

  /**
   * 开始定期健康检查
   */
  startHealthCheck(intervalMs: number = 10000): void {
    if (this.healthCheckInterval) {
      this.stopHealthCheck();
    }

    this.healthCheckInterval = window.setInterval(async () => {
      if (this.isChecking) return;
      
      this.isChecking = true;
      await this.performHealthCheck();
      this.isChecking = false;
    }, intervalMs);

    // 立即执行一次检查
    this.performHealthCheck();
  }

  /**
   * 停止健康检查
   */
  stopHealthCheck(): void {
    if (this.healthCheckInterval) {
      clearInterval(this.healthCheckInterval);
      this.healthCheckInterval = null;
    }
  }

  /**
   * 执行健康检查并更新状态
   */
  private async performHealthCheck(): Promise<void> {
    const systemStore = useSystemStore.getState();
    
    try {
      // 检查API健康状态
      const healthResponse = await this.checkHealth();
      
      if (healthResponse && healthResponse.success) {
        // API连接正常
        systemStore.updateConnectionStatus('api', true);
        
        // 检查ROS状态
        const rosReady = healthResponse.ros_ready;
        systemStore.updateConnectionStatus('ros', rosReady);
        
        console.log('✅ API健康检查成功:', {
          api: true,
          ros: rosReady,
          websocket_clients: healthResponse.websocket_clients,
          baseURL: this.baseURL
        });
      } else {
        // API连接失败
        systemStore.updateConnectionStatus('api', false);
        systemStore.updateConnectionStatus('ros', false);
        console.log('❌ API健康检查失败, baseURL:', this.baseURL);
      }
    } catch (error) {
      console.error('健康检查执行失败:', error);
      systemStore.updateConnectionStatus('api', false);
      systemStore.updateConnectionStatus('ros', false);
    }
  }

  /**
   * 手动更新API基础URL（兼容旧接口）
   */
  updateBaseURLManual(host?: string, port?: number): void {
    if (host && port) {
      // 如果提供了具体参数，使用参数
      this.baseURL = `http://${host}:${port}`;
    } else {
      // 否则动态获取
      this.updateBaseURL();
    }
    console.log('🔧 API基础URL已手动更新:', this.baseURL);
  }

  /**
   * 获取当前API基础URL
   */
  getBaseURL(): string {
    return this.baseURL;
  }
}

// 创建全局API服务实例
export const apiService = new APIService();

// 导出默认实例
export default apiService;
