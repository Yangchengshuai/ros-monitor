// WebSocket消息类型定义

export interface WSMessage {
  type: string;
  timestamp: string;
  data: any;
}

export interface WSSubscribeMessage extends WSMessage {
  type: 'subscribe';
  data: {
    topics: string[];
    frequency?: number;
  };
}

export interface WSUnsubscribeMessage extends WSMessage {
  type: 'unsubscribe';
  data: {
    topics: string[];
  };
}

export interface WSConnectedMessage extends WSMessage {
  type: 'connected';
  client_id: string;
}

export interface WSSubscribedMessage extends WSMessage {
  type: 'subscribed';
  topics: string[];
}

export interface WSIMUMessage extends WSMessage {
  type: 'imu';
  data: {
    timestamp: number;
    data: {
      timestamp: number;
      orientation: {
        x: number; y: number; z: number; w: number;
      };
      angular_velocity: {
        x: number; y: number; z: number;
      };
      linear_acceleration: {
        x: number; y: number; z: number;
      };
    };
  };
}

export interface WSCameraMessage extends WSMessage {
  type: 'camera';
  data: {
    timestamp: number;
    data: {
      camera_id: string;
      timestamp: number;
      sequence: number;
      encoding: string;
      width: number;
      height: number;
      data: string;
      compressed: boolean;
    };
  };
}

export interface WSErrorMessage extends WSMessage {
  type: 'error';
  data: {
    message: string;
    code?: number;
  };
}

