import { getApiBaseUrl } from '../utils/network';

export interface APIResponse {
  success: boolean;
  message: string;
  data?: any;
}

export interface StartRequest {
  script: string;
  timeout: number;
}

export interface StopRequest {
  force?: boolean;
}

class DataCollectionService {
  private baseURL: string = '';

  constructor() {
    // 动态获取当前主机地址和端口
    this.updateBaseURL();
  }

  /**
   * 动态更新API基础URL
   */
  private updateBaseURL(): void {
    this.baseURL = getApiBaseUrl();
    console.log('🔧 数据采集服务API基础URL已更新:', this.baseURL);
  }

  /**
   * 启动数据采集
   */
  async startCollection(): Promise<APIResponse> {
    try {
      // 每次请求前更新地址
      this.updateBaseURL();
      
      const response = await fetch(`${this.baseURL}/api/v1/data-collection/start`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          script: 'start_all.sh',
          timeout: 30
        })
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      return await response.json();
    } catch (error) {
      return {
        success: false,
        message: `启动失败: ${error instanceof Error ? error.message : '未知错误'}`
      };
    }
  }

  /**
   * 停止数据采集
   */
  async stopCollection(): Promise<APIResponse> {
    try {
      // 每次请求前更新地址
      this.updateBaseURL();
      
      const response = await fetch(`${this.baseURL}/api/v1/data-collection/stop`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          force: false
        })
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      return await response.json();
    } catch (error) {
      return {
        success: false,
        message: `停止失败: ${error instanceof Error ? error.message : '未知错误'}`
      };
    }
  }

  /**
   * 获取数据采集状态
   */
  async getStatus(): Promise<APIResponse> {
    try {
      // 每次请求前更新地址
      this.updateBaseURL();
      
      const response = await fetch(`${this.baseURL}/api/v1/data-collection/status`, {
        method: 'GET',
        headers: {
          'Content-Type': 'application/json',
        }
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      return await response.json();
    } catch (error) {
      return {
        success: false,
        message: `获取状态失败: ${error instanceof Error ? error.message : '未知错误'}`
      };
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
    console.log('🔧 数据采集服务API基础URL已手动更新:', this.baseURL);
  }

  /**
   * 获取当前API基础URL
   */
  getBaseURL(): string {
    return this.baseURL;
  }
}

// 创建全局数据采集服务实例
export const dataCollectionService = new DataCollectionService();

// 导出默认实例
export default dataCollectionService;