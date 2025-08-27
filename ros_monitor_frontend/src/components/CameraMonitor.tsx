import React, { useState, useEffect } from 'react';
import { Row, Col, Typography, Space, Card, Statistic, Alert } from 'antd';
import { CameraOutlined, ClockCircleOutlined, InfoCircleOutlined } from '@ant-design/icons';
import CameraViewer from './CameraViewer';
import { wsService } from '../services/websocket';
import { getConnectionInfo, getNetworkAccessTip } from '../utils/network';

const { Title, Text } = Typography;

export const CameraMonitor: React.FC = () => {
  const [lastMessage] = useState<any>(null);
  const [wsConnected, setWsConnected] = useState(false);
  const [wsError, setWsError] = useState<string | null>(null);

  // 在组件挂载时建立全局WebSocket连接
  useEffect(() => {
    const connectWebSocket = async () => {
      try {
        console.log('CameraMonitor: 开始建立全局WebSocket连接...');
        await wsService.connect();
        setWsConnected(true);
        setWsError(null);
        console.log('CameraMonitor: 全局WebSocket连接成功');
      } catch (error) {
        console.error('CameraMonitor: 全局WebSocket连接失败:', error);
        setWsError(error instanceof Error ? error.message : '连接失败');
        setWsConnected(false);
      }
    };

    // 如果还没有连接，则建立连接
    if (!wsService.isConnected()) {
      connectWebSocket();
    } else {
      setWsConnected(true);
      console.log('CameraMonitor: WebSocket已连接');
    }

    // 清理函数
    return () => {
      // 注意：这里不要断开连接，因为其他组件可能还在使用
      console.log('CameraMonitor: 组件卸载，保持WebSocket连接');
    };
  }, []);

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
              prefix={<CameraOutlined />}
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
              title="相机数量"
              value="2"
              prefix={<CameraOutlined />}
            />
          </Card>
        </Col>
      </Row>

      {/* 网络连接信息 */}
      <Row gutter={[16, 16]} style={{ marginBottom: 16 }}>
        <Col span={24}>
          <Alert
            message="网络连接信息"
            description={
              <div>
                <p><strong>当前状态:</strong> {getConnectionInfo().displayText}</p>
                <p><strong>WebSocket地址:</strong> {getConnectionInfo().wsUrl}</p>
                <p><strong>API地址:</strong> {getConnectionInfo().apiUrl}</p>
                <p><strong>提示:</strong> {getNetworkAccessTip()}</p>
              </div>
            }
            type="info"
            showIcon
            icon={<InfoCircleOutlined />}
          />
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
          <Text>2. 全局WebSocket连接由CameraMonitor管理</Text>
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