/**
 * 数据采集控制API服务
 * 最小化实现，遵循YAGNI和KISS原则
 */

export interface DataCollectionStatus {
  is_running: boolean;
  process_id: number | null;
  start_time: number | null;
  script_path: string;
  last_update: number;
}

export interface APIResponse<T = any> {
  success: boolean;
  message: string;
  data?: T;
}

class DataCollectionService {
  private baseURL: string;

  constructor(host: string = 'localhost', port: number = 8000) {
    this.baseURL = `http://${host}:${port}`;
  }

  /**
   * 启动数据采集
   */
  async startCollection(): Promise<APIResponse> {
    try {
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
      const response = await fetch(`${this.baseURL}/api/v1/data-collection/stop`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ force: false })
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
   * 获取采集状态
   */
  async getStatus(): Promise<APIResponse<DataCollectionStatus>> {
    try {
      const response = await fetch(`${this.baseURL}/api/v1/data-collection/status`);

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
}

export const dataCollectionService = new DataCollectionService();