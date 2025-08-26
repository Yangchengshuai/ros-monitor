import React, { useEffect, useState } from 'react';
import { Row, Col, Typography, Space, Card, Statistic, Button } from 'antd';
import { CameraOutlined, WifiOutlined, ClockCircleOutlined, ReloadOutlined } from '@ant-design/icons';
import CameraViewer from './CameraViewer';
import { WebSocketService } from '../services/websocket';

const { Title, Text } = Typography;

export const CameraMonitor: React.FC = () => {
  const [wsConnected, setWsConnected] = useState(false);
  const [wsError, setWsError] = useState<string | null>(null);
  const [lastMessage] = useState<any>(null);
  const [wsService] = useState(() => new WebSocketService());

  useEffect(() => {
    // 连接WebSocket
    const connectWebSocket = async () => {
      try {
        await wsService.connect('localhost', 8000);
        setWsConnected(true);
        setWsError(null);
        console.log('WebSocket连接成功');
      } catch (error) {
        console.error('WebSocket连接失败:', error);
        setWsError(error instanceof Error ? error.message : '连接失败');
        setWsConnected(false);
      }
    };

    connectWebSocket();

    // 清理函数
    return () => {
      wsService.disconnect();
    };
  }, [wsService]);

  const handleReconnect = async () => {
    setWsError(null);
    try {
      await wsService.connect('localhost', 8000);
      setWsConnected(true);
      setWsError(null);
    } catch (error) {
      setWsError(error instanceof Error ? error.message : '重连失败');
      setWsConnected(false);
    }
  };

  return (
    <div style={{ padding: 24 }}>
      <div style={{ marginBottom: 24 }}>
        <Title level={2}>
          <CameraOutlined style={{ marginRight: 8 }} />
          相机监控系统
        </Title>
        <Text type="secondary">
          实时监控双目相机数据流，支持图像预览、设置调整和数据下载
        </Text>
      </div>

      {/* 连接状态卡片 */}
      <Row gutter={[16, 16]} style={{ marginBottom: 24 }}>
        <Col span={8}>
          <Card>
            <Statistic
              title="WebSocket连接状态"
              value={wsConnected ? '已连接' : '未连接'}
              valueStyle={{ color: wsConnected ? '#3f8600' : '#cf1322' }}
              prefix={<WifiOutlined />}
            />
          </Card>
        </Col>
        <Col span={8}>
          <Card>
            <Statistic
              title="最后消息时间"
              value={lastMessage ? new Date().toLocaleTimeString() : '无数据'}
              prefix={<ClockCircleOutlined />}
            />
          </Card>
        </Col>
        <Col span={8}>
          <Card>
            <Statistic
              title="系统状态"
              value={wsError ? '错误' : '正常'}
              valueStyle={{ color: wsError ? '#cf1322' : '#3f8600' }}
              prefix={<CameraOutlined />}
            />
          </Card>
        </Col>
      </Row>

      {/* 连接控制 */}
      <Row gutter={[16, 16]} style={{ marginBottom: 24 }}>
        <Col span={24}>
          <Card>
            <Space>
              <Button 
                type="primary" 
                icon={<ReloadOutlined />}
                onClick={handleReconnect}
                loading={!wsConnected && !wsError}
              >
                {wsConnected ? '重新连接' : '连接'}
              </Button>
              <Text type="secondary">
                WebSocket服务地址: ws://localhost:8000
              </Text>
            </Space>
          </Card>
        </Col>
      </Row>

      {/* 错误提示 */}
      {wsError && (
        <Card style={{ marginBottom: 24, borderColor: '#ff4d4f' }}>
          <Text type="danger">
            <strong>连接错误:</strong> {wsError}
          </Text>
        </Card>
      )}

      {/* 相机视图 */}
      <Row gutter={[24, 24]}>
        <Col span={12}>
          <CameraViewer 
            cameraId="left_camera" 
            className="camera-viewer"
          />
        </Col>
        <Col span={12}>
          <CameraViewer 
            cameraId="right_camera" 
            className="camera-viewer"
          />
        </Col>
      </Row>

      {/* 使用说明 */}
      <Card style={{ marginTop: 24 }}>
        <Title level={4}>使用说明</Title>
        <Space direction="vertical" size="small">
          <Text>1. 确保后端服务已启动并正常运行</Text>
          <Text>2. 点击"连接"按钮建立WebSocket连接</Text>
          <Text>3. 点击相机开关开启数据流</Text>
          <Text>4. 使用设置按钮调整图像缩放比例</Text>
          <Text>5. 支持全屏查看和图像下载</Text>
          <Text>6. 实时显示帧率、压缩率等统计信息</Text>
        </Space>
      </Card>
    </div>
  );
};

export default CameraMonitor;