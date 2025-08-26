import { useSystemStore } from '../stores/useSystemStore';

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
  private baseURL: string;
  private healthCheckInterval: number | null = null;
  private isChecking = false;

  constructor(host: string = 'localhost', port: number = 8000) {
    this.baseURL = `http://${host}:${port}`;
  }

  /**
   * 检查API健康状态
   */
  async checkHealth(): Promise<APIHealthResponse | null> {
    try {
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
          websocket_clients: healthResponse.websocket_clients
        });
      } else {
        // API连接失败
        systemStore.updateConnectionStatus('api', false);
        systemStore.updateConnectionStatus('ros', false);
        console.log('❌ API健康检查失败');
      }
    } catch (error) {
      console.error('健康检查执行失败:', error);
      systemStore.updateConnectionStatus('api', false);
      systemStore.updateConnectionStatus('ros', false);
    }
  }

  /**
   * 更新API基础URL
   */
  updateBaseURL(host: string, port: number): void {
    this.baseURL = `http://${host}:${port}`;
    console.log('API基础URL已更新:', this.baseURL);
  }
}

// 创建全局API服务实例
export const apiService = new APIService();

// 导出默认实例
export default apiService;
