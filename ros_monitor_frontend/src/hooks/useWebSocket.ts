import { useState, useEffect, useRef, useCallback } from 'react';
import { getWebSocketUrl } from '../utils/network';

interface WebSocketMessage {
  type: string;
  [key: string]: any;
}

interface UseWebSocketReturn {
  connected: boolean;
  error: string | null;
  sendMessage: (message: WebSocketMessage) => void;
  lastMessage: WebSocketMessage | null;
}

export const useWebSocket = (url?: string): UseWebSocketReturn => {
  // 如果没有指定url，使用动态获取的地址
  const defaultUrl = url || getWebSocketUrl();
  const [connected, setConnected] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [lastMessage, setLastMessage] = useState<WebSocketMessage | null>(null);
  const wsRef = useRef<WebSocket | null>(null);
  const reconnectTimeoutRef = useRef<number | null>(null);
  const reconnectAttempts = useRef(0);
  const maxReconnectAttempts = 5;
  const reconnectDelay = 3000;

  const connect = useCallback(() => {
    try {
      const clientId = `frontend_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
      const wsUrl = `${defaultUrl}/${clientId}`;
      
      console.log('useWebSocket connecting to:', wsUrl);
      
      const ws = new WebSocket(wsUrl);
      wsRef.current = ws;

      ws.onopen = () => {
        console.log('WebSocket connected');
        setConnected(true);
        setError(null);
        reconnectAttempts.current = 0;
      };

      ws.onmessage = (event) => {
        try {
          const message = JSON.parse(event.data);
          setLastMessage(message);
          
          // 处理不同类型的消息
          handleMessage(message);
          
          // 触发自定义事件，让组件能够接收到消息
          const customEvent = new CustomEvent('websocket-message', {
            detail: message
          });
          window.dispatchEvent(customEvent);
        } catch (error) {
          console.error('Failed to parse WebSocket message:', error);
        }
      };

      ws.onclose = () => {
        console.log('WebSocket disconnected');
        setConnected(false);
        attemptReconnect();
      };

      ws.onerror = (error) => {
        console.error('WebSocket error:', error);
        setError('WebSocket连接错误');
      };

    } catch (error) {
      console.error('Failed to create WebSocket:', error);
      setError('创建WebSocket连接失败');
    }
  }, [defaultUrl]);

  const attemptReconnect = useCallback(() => {
    if (reconnectAttempts.current < maxReconnectAttempts) {
      reconnectAttempts.current++;
      console.log(`Attempting to reconnect... (${reconnectAttempts.current}/${maxReconnectAttempts})`);
      
      reconnectTimeoutRef.current = setTimeout(() => {
        connect();
      }, reconnectDelay);
    } else {
      setError('WebSocket重连失败，已达到最大重试次数');
    }
  }, [connect]);

  const disconnect = useCallback(() => {
    if (wsRef.current) {
      wsRef.current.close();
      wsRef.current = null;
    }
    
    if (reconnectTimeoutRef.current) {
      clearTimeout(reconnectTimeoutRef.current);
      reconnectTimeoutRef.current = null;
    }
    
    setConnected(false);
  }, []);

  const sendMessage = useCallback((message: WebSocketMessage) => {
    if (wsRef.current && wsRef.current.readyState === WebSocket.OPEN) {
      wsRef.current.send(JSON.stringify(message));
    } else {
      console.error('WebSocket is not connected');
      setError('WebSocket未连接，无法发送消息');
    }
  }, []);

  const handleMessage = useCallback((message: WebSocketMessage) => {
    // 根据消息类型处理不同的逻辑
    switch (message.type) {
      case 'connected':
        console.log('WebSocket connection confirmed');
        break;
      case 'error':
        console.error('WebSocket error message:', message.message);
        setError(message.message);
        break;
      case 'subscription_confirmed':
        console.log('Topic subscription confirmed:', message.topics);
        break;
      case 'system_status':
        console.log('System status received:', message);
        break;
      default:
        // 其他消息类型可以在这里处理
        break;
    }
  }, []);

  useEffect(() => {
    connect();

    return () => {
      disconnect();
    };
  }, [connect, disconnect]);

  // 清理重连定时器
  useEffect(() => {
    return () => {
      if (reconnectTimeoutRef.current) {
        clearTimeout(reconnectTimeoutRef.current);
      }
    };
  }, []);

  return {
    connected,
    error,
    sendMessage,
    lastMessage
  };
};