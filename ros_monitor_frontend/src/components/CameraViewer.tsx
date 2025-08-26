import React, { useEffect, useRef, useState, useCallback } from 'react';
import { Card, Switch, Slider, Button, Space, Tag, Tooltip, message } from 'antd';
import { 
  FullscreenOutlined, 
  DownloadOutlined, 
  SettingOutlined,
  CameraOutlined,
  ReloadOutlined
} from '@ant-design/icons';
import { useWebSocket } from '../hooks/useWebSocket';

interface CameraData {
  camera_id: string;
  topic: string;
  timestamp: number;
  sequence: number;
  encoding: string;
  width: number;
  height: number;
  data: string;
  compressed: boolean;
  compressed_size: number;
  compression_ratio: number;
  frame_rate: number;
}

interface CameraViewerProps {
  cameraId: 'left_camera' | 'right_camera';
  className?: string;
}

export const CameraViewer: React.FC<CameraViewerProps> = ({ 
  cameraId, 
  className 
}) => {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const [isStreaming, setIsStreaming] = useState(false);
  const [imageScale, setImageScale] = useState(1.0);
  const [showSettings, setShowSettings] = useState(false);
  const [cameraData, setCameraData] = useState<CameraData | null>(null);
  const [lastUpdateTime, setLastUpdateTime] = useState<number>(0);
  const [frameCount, setFrameCount] = useState(0);
  const [errorCount, setErrorCount] = useState(0);
  
  const { connected, sendMessage } = useWebSocket();
  
  // 相机标题和状态
  const cameraTitle = cameraId === 'left_camera' ? '左相机' : '右相机';
  const connectionStatus = cameraData ? 'connected' : 'disconnected';
  
  // 处理WebSocket消息
  useEffect(() => {
    if (!connected) return;
    
    // 订阅相机数据
    sendMessage({
      type: 'subscribe',
      topics: ['camera']
    });
    
    // 设置消息处理函数
    const handleWebSocketMessage = (message: any) => {
      try {
        if (message.type === 'camera' && message.camera_id === cameraId) {
          console.log(`收到相机数据: ${cameraId}`, message);
          setCameraData(message.data);
          setLastUpdateTime(Date.now());
          setFrameCount(prev => prev + 1);
          setErrorCount(0);
        }
      } catch (error) {
        console.error('Failed to parse WebSocket message:', error);
        setErrorCount(prev => prev + 1);
      }
    };
    
    // 使用自定义事件来传递WebSocket消息
    const messageHandler = (event: CustomEvent) => {
      handleWebSocketMessage(event.detail);
    };
    
    window.addEventListener('websocket-message', messageHandler as EventListener);
    
    return () => {
      window.removeEventListener('websocket-message', messageHandler as EventListener);
    };
  }, [connected, cameraId, sendMessage]);
  
  // 订阅/取消订阅相机数据
  useEffect(() => {
    if (!connected) return;
    
    if (isStreaming) {
      // 订阅相机数据
      sendMessage({
        type: 'subscribe',
        topics: ['camera']
      });
      message.success(`${cameraTitle} 数据流已开启`);
    } else {
      // 取消订阅
      sendMessage({
        type: 'unsubscribe',
        topics: ['camera']
      });
      message.info(`${cameraTitle} 数据流已停止`);
    }
  }, [isStreaming, connected, sendMessage, cameraTitle]);
  
  // 绘制图像到Canvas
  useEffect(() => {
    if (cameraData?.data && canvasRef.current) {
      drawImage();
    }
  }, [cameraData?.data, imageScale]);
  
  const drawImage = useCallback(() => {
    const canvas = canvasRef.current;
    const ctx = canvas?.getContext('2d');
    
    if (!canvas || !ctx || !cameraData?.data) return;
    
    const img = new Image();
    img.onload = () => {
      // 调整canvas尺寸
      const scaledWidth = img.width * imageScale;
      const scaledHeight = img.height * imageScale;
      
      canvas.width = scaledWidth;
      canvas.height = scaledHeight;
      
      // 绘制图像
      ctx.clearRect(0, 0, scaledWidth, scaledHeight);
      ctx.drawImage(img, 0, 0, scaledWidth, scaledHeight);
      
      // 叠加信息
      drawOverlay(ctx);
    };
    
    img.src = `data:image/${cameraData.encoding};base64,${cameraData.data}`;
  }, [cameraData, imageScale]);
  
  const drawOverlay = useCallback((ctx: CanvasRenderingContext2D) => {
    // 绘制时间戳和状态信息
    ctx.fillStyle = 'rgba(0, 0, 0, 0.7)';
    ctx.fillRect(10, 10, 300, 100);
    
    ctx.fillStyle = 'white';
    ctx.font = '14px Arial';
    ctx.fillText(`Camera: ${cameraTitle}`, 20, 30);
    ctx.fillText(`Timestamp: ${new Date((cameraData?.timestamp || Date.now() / 1000) * 1000).toLocaleTimeString()}`, 20, 50);
    ctx.fillText(`Resolution: ${cameraData?.width || 0}x${cameraData?.height || 0}`, 20, 70);
    ctx.fillText(`Frame: ${frameCount} | FPS: ${cameraData?.frame_rate || 0}`, 20, 90);
  }, [cameraTitle, cameraData, frameCount]);
  
  // 事件处理函数
  const handleStreamToggle = useCallback((checked: boolean) => {
    setIsStreaming(checked);
  }, []);
  
  const handleScaleChange = useCallback((value: number) => {
    setImageScale(value);
  }, []);
  
  const handleFullscreen = useCallback(() => {
    const canvas = canvasRef.current;
    if (canvas && canvas.requestFullscreen) {
      canvas.requestFullscreen();
    }
  }, []);
  
  const handleDownload = useCallback(() => {
    const canvas = canvasRef.current;
    if (canvas) {
      const link = document.createElement('a');
      link.download = `${cameraId}_${Date.now()}.png`;
      link.href = canvas.toDataURL();
      link.click();
    }
  }, [cameraId]);
  
  const handleRefresh = useCallback(() => {
    if (connected) {
      sendMessage({
        type: 'request_system_status'
      });
      message.info('正在刷新系统状态...');
    }
  }, [connected, sendMessage]);
  
  // 计算状态信息
  const statusColor = connectionStatus === 'connected' ? 'success' : 'error';
  const statusText = connectionStatus === 'connected' ? '已连接' : '未连接';
  const lastUpdateText = lastUpdateTime > 0 ? 
    new Date(lastUpdateTime).toLocaleTimeString() : '无数据';
  
  return (
    <Card
      title={
        <div style={{ display: 'flex', alignItems: 'center', gap: 8 }}>
          <CameraOutlined />
          <span>{cameraTitle}</span>
          <Tag color={statusColor}>{statusText}</Tag>
        </div>
      }
      extra={
        <Space>
          <Tooltip title="连接状态">
            <div 
              style={{ 
                width: 8, 
                height: 8, 
                borderRadius: '50%',
                backgroundColor: statusColor === 'success' ? '#52c41a' : '#ff4d4f'
              }} 
            />
          </Tooltip>
          <Switch
            checked={isStreaming}
            onChange={handleStreamToggle}
            checkedChildren="ON"
            unCheckedChildren="OFF"
            disabled={!connected}
          />
          <Button 
            icon={<SettingOutlined />}
            onClick={() => setShowSettings(!showSettings)}
            size="small"
          />
          <Button 
            icon={<ReloadOutlined />}
            onClick={handleRefresh}
            size="small"
          />
          <Button 
            icon={<FullscreenOutlined />}
            onClick={handleFullscreen}
            size="small"
          />
          <Button 
            icon={<DownloadOutlined />}
            onClick={handleDownload}
            size="small"
          />
        </Space>
      }
      className={className}
    >
      <div style={{ position: 'relative' }}>
        <canvas
          ref={canvasRef}
          style={{
            width: '100%',
            height: 'auto',
            border: '1px solid #d9d9d9',
            borderRadius: 4,
            backgroundColor: '#f0f0f0'
          }}
        />
        
        {/* 设置面板 */}
        {showSettings && (
          <div 
            style={{
              position: 'absolute',
              top: 10,
              right: 10,
              background: 'rgba(0, 0, 0, 0.8)',
              padding: 16,
              borderRadius: 4,
              color: 'white',
              minWidth: 200
            }}
          >
            <div style={{ marginBottom: 16 }}>
              <label>缩放比例: {imageScale.toFixed(1)}</label>
              <Slider
                min={0.1}
                max={2.0}
                step={0.1}
                value={imageScale}
                onChange={handleScaleChange}
                style={{ marginTop: 8 }}
              />
            </div>
          </div>
        )}
        
        {/* 无数据提示 */}
        {!isStreaming && (
          <div 
            style={{
              position: 'absolute',
              top: '50%',
              left: '50%',
              transform: 'translate(-50%, -50%)',
              textAlign: 'center',
              color: '#999'
            }}
          >
            <CameraOutlined style={{ fontSize: 48, marginBottom: 16 }} />
            <p>相机流已停止</p>
            <p>请开启开关以查看实时画面</p>
          </div>
        )}
        
        {/* 连接状态提示 */}
        {!connected && (
          <div 
            style={{
              position: 'absolute',
              top: '50%',
              left: '50%',
              transform: 'translate(-50%, -50%)',
              textAlign: 'center',
              color: '#ff4d4f'
            }}
          >
            <p>WebSocket未连接</p>
            <p>请检查网络连接</p>
          </div>
        )}
      </div>
      
      {/* 状态信息 */}
      <div style={{ marginTop: 16, fontSize: 12, color: '#666' }}>
        <Space split={<span>|</span>}>
          <span>最后更新: {lastUpdateText}</span>
          <span>帧数: {frameCount}</span>
          <span>错误: {errorCount}</span>
          {cameraData && (
            <>
              <span>压缩率: {cameraData.compression_ratio}%</span>
              <span>帧率: {cameraData.frame_rate} FPS</span>
            </>
          )}
        </Space>
      </div>
    </Card>
  );
};

export default CameraViewer;