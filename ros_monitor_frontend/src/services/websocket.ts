import { useSensorStore } from '../stores/useSensorStore';
import { useSystemStore } from '../stores/useSystemStore';
import { getCurrentHost } from '../utils/network';

export interface WSMessage {
  type: string;
  timestamp: string;
  data: any;
}

export class WebSocketService {
  private ws: WebSocket | null = null;
  private reconnectAttempts = 0;
  private maxReconnectAttempts = 5;
  private reconnectInterval = 3000;
  private clientId: string;
  private url: string = '';
  private isConnecting = false;
  
  constructor(clientId?: string) {
    this.clientId = clientId || this.generateClientId();
  }
  
  private generateClientId(): string {
    return `frontend_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
  }
  
  connect(host?: string, port?: number): Promise<void> {
    if (this.isConnecting) {
      return Promise.reject(new Error('Connection already in progress'));
    }
    
    // 如果没有指定host，自动获取当前主机地址
    const targetHost = host || getCurrentHost();
    
    // 如果没有指定port，从环境变量获取或使用默认值
    const targetPort = port || import.meta.env.VITE_WS_PORT || 8000;
    
    this.isConnecting = true;
    this.url = `ws://${targetHost}:${targetPort}/ws/${this.clientId}`;
    
    return new Promise((resolve, reject) => {
      try {
        console.log('Connecting to WebSocket:', this.url);
        this.ws = new WebSocket(this.url);
        
        this.ws.onopen = () => {
          console.log('✅ WebSocket connected successfully');
          this.isConnecting = false;
          this.reconnectAttempts = 0;
          
          // 发送认证消息
          const token = localStorage.getItem('access_token');
          if (token) {
            this.send({
              type: 'auth',
              timestamp: new Date().toISOString(),
              data: { token }
            });
          }
          
          // 更新系统状态
          useSystemStore.getState().updateConnectionStatus('websocket', true);
          
          resolve();
        };
        
        this.ws.onmessage = (event) => {
          try {
            const message: WSMessage = JSON.parse(event.data);
            this.handleMessage(message);
          } catch (error) {
            console.error('Failed to parse WebSocket message:', error);
          }
        };
        
        this.ws.onclose = (event) => {
          console.log('WebSocket disconnected:', event.code, event.reason);
          this.isConnecting = false;
          useSystemStore.getState().updateConnectionStatus('websocket', false);
          this.attemptReconnect();
        };
        
        this.ws.onerror = (error) => {
          console.error('WebSocket error:', error);
          this.isConnecting = false;
          reject(error);
        };
        
      } catch (error) {
        this.isConnecting = false;
        reject(error);
      }
    });
  }
  
  private handleMessage(message: WSMessage): void {
    const { type, data } = message;
    
    switch (type) {
      case 'connected':
        console.log('WebSocket authentication successful');
        break;
        
      case 'subscribed':
        console.log('Topics subscribed:', data.topics);
        break;
        
      case 'subscription_confirmed':
        console.log('Subscription confirmed:', data);
        break;
        
      case 'camera':
        // 修复：相机数据在后端的data字段中，需要正确解构
        if (data && data.camera_id) {
          this.handleCameraData(data);
          // 触发自定义事件，让CameraViewer组件能够接收到数据
          const customEvent = new CustomEvent('websocket-message', {
            detail: message
          });
          window.dispatchEvent(customEvent);
        } else {
          console.warn('Invalid camera data format:', data);
        }
        break;
        
      case 'lidar':
        if (data) {
          this.handleLidarData(data);
        } else {
          console.warn('Invalid lidar data format:', message);
        }
        break;
        
      case 'system_status':
        if (data) {
          this.handleSystemStatus(data);
        } else {
          console.warn('Invalid system status format:', message);
        }
        break;
        
      case 'ack':
        console.log('Message acknowledged:', data);
        break;
        
      case 'error':
        console.error('WebSocket error message:', data);
        break;
        
      default:
        console.warn('Unknown message type:', type, 'with data:', data);
    }
  }
  

  
  private handleCameraData(data: any): void {
    const sensorStore = useSensorStore.getState();
    if (data.camera_id) {
      sensorStore.updateCameraData(data.camera_id, {
        camera_id: data.camera_id,
        timestamp: data.timestamp,
        sequence: data.sequence || 0,
        encoding: data.encoding,
        width: data.width,
        height: data.height,
        data: data.data,
        compressed: data.compressed || false
      });
    }
  }
  
  private handleLidarData(data: any): void {
    const sensorStore = useSensorStore.getState();
    sensorStore.updateLidarData({
      timestamp: data.timestamp,
      frame_id: data.frame_id || 'map',
      point_count: data.point_count,
      points: data.data
    });
  }
  
  private handleSystemStatus(data: any): void {
    const systemStore = useSystemStore.getState();
    systemStore.updatePerformanceMetrics({
      dataRate: data.data_rate || 0,
      errorCount: data.error_count || 0
    });
  }
  
  send(message: WSMessage): void {
    if (this.ws && this.ws.readyState === WebSocket.OPEN) {
      this.ws.send(JSON.stringify(message));
    } else {
      console.error('WebSocket is not connected');
    }
  }
  
  subscribe(topics: string[]): void {
    this.send({
      type: 'subscribe',
      timestamp: new Date().toISOString(),
      data: { topics }
    });
  }
  
  unsubscribe(topics: string[]): void {
    this.send({
      type: 'unsubscribe',
      timestamp: new Date().toISOString(),
      data: { topics }
    });
  }
  
  private attemptReconnect(): void {
    if (this.reconnectAttempts < this.maxReconnectAttempts) {
      this.reconnectAttempts++;
      console.log(`Attempting to reconnect... (${this.reconnectAttempts}/${this.maxReconnectAttempts})`);
      
      setTimeout(() => {
        if (this.url) {
          this.connect(this.url.replace('ws://', '').split(':')[0], 
                      parseInt(this.url.split(':')[2].split('/')[0]))
            .catch(console.error);
        }
      }, this.reconnectInterval);
    } else {
      console.error('Max reconnection attempts reached');
    }
  }
  
  disconnect(): void {
    if (this.ws) {
      this.ws.close();
      this.ws = null;
    }
  }
  
  isConnected(): boolean {
    return this.ws?.readyState === WebSocket.OPEN;
  }
}

// 单例WebSocket服务
export const wsService = new WebSocketService();
