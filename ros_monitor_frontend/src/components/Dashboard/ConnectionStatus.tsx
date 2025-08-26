import React from 'react';
import { Card, Row, Col, Statistic, Tag, Space, Progress } from 'antd';
import { 
  CheckCircleOutlined,
  CloseCircleOutlined
} from '@ant-design/icons';
import { useWebSocket } from '../../hooks/useWebSocket';
import { useSystemStore } from '../../stores/useSystemStore';

interface ConnectionStatusProps {
  className?: string;
}

export const ConnectionStatus: React.FC<ConnectionStatusProps> = ({ className }) => {
  const { connected: wsConnected } = useWebSocket();
  const { connection, performance } = useSystemStore();
  
  const getConnectionIcon = (status: boolean) => {
    if (status) {
      return <CheckCircleOutlined style={{ color: '#52c41a' }} />;
    }
    return <CloseCircleOutlined style={{ color: '#ff4d4f' }} />;
  };
  
  const getConnectionColor = (status: boolean) => {
    return status ? 'green' : 'red';
  };
  
  const getConnectionText = (status: boolean, connecting: boolean = false) => {
    if (connecting) return '连接中...';
    return status ? '已连接' : '未连接';
  };
  
  const getLatencyColor = (latency: number) => {
    if (latency < 100) return '#52c41a';
    if (latency < 300) return '#faad14';
    return '#ff4d4f';
  };
  
  const getDataRateColor = (rate: number) => {
    if (rate > 50) return '#52c41a';
    if (rate > 20) return '#faad14';
    return '#ff4d4f';
  };
  
  return (
    <Card title="连接状态监控" className={className}>
      <Row gutter={[16, 16]}>
        {/* WebSocket连接状态 */}
        <Col span={8}>
          <Card size="small" title="WebSocket连接" bordered={false}>
            <Space direction="vertical" style={{ width: '100%' }}>
              <div style={{ textAlign: 'center' }}>
                {getConnectionIcon(wsConnected)}
                <div style={{ marginTop: 8 }}>
                  <Tag color={getConnectionColor(wsConnected)}>
                    {getConnectionText(wsConnected)}
                  </Tag>
                </div>
              </div>
              <Statistic
                title="延迟"
                value={performance.wsLatency}
                suffix="ms"
                valueStyle={{ 
                  color: getLatencyColor(performance.wsLatency),
                  fontSize: '16px'
                }}
              />
            </Space>
          </Card>
        </Col>
        
        {/* 后端API连接状态 */}
        <Col span={8}>
          <Card size="small" title="后端API" bordered={false}>
            <Space direction="vertical" style={{ width: '100%' }}>
              <div style={{ textAlign: 'center' }}>
                {getConnectionIcon(connection.api)}
                <div style={{ marginTop: 8 }}>
                  <Tag color={getConnectionColor(connection.api)}>
                    {getConnectionText(connection.api)}
                  </Tag>
                </div>
              </div>
              <Statistic
                title="响应时间"
                value={performance.wsLatency}
                suffix="ms"
                valueStyle={{ 
                  color: getLatencyColor(performance.wsLatency),
                  fontSize: '16px'
                }}
              />
            </Space>
          </Card>
        </Col>
        
        {/* ROS连接状态 */}
        <Col span={8}>
          <Card size="small" title="ROS系统" bordered={false}>
            <Space direction="vertical" style={{ width: '100%' }}>
              <div style={{ textAlign: 'center' }}>
                {getConnectionIcon(connection.ros)}
                <div style={{ marginTop: 8 }}>
                  <Tag color={getConnectionColor(connection.ros)}>
                    {getConnectionText(connection.ros)}
                  </Tag>
                </div>
              </div>
              <Statistic
                title="节点数量"
                value={connection.ros ? '运行中' : '0'}
                valueStyle={{ 
                  color: connection.ros ? '#52c41a' : '#ff4d4f',
                  fontSize: '16px'
                }}
              />
            </Space>
          </Card>
        </Col>
        
        {/* 数据传输统计 */}
        <Col span={12}>
          <Card size="small" title="数据传输统计" bordered={false}>
            <Row gutter={[16, 8]}>
              <Col span={12}>
                <Statistic
                  title="数据速率"
                  value={performance.dataRate}
                  suffix="Hz"
                  valueStyle={{ 
                    color: getDataRateColor(performance.dataRate),
                    fontSize: '16px'
                  }}
                />
              </Col>
              <Col span={12}>
                <Statistic
                  title="错误计数"
                  value={performance.errorCount}
                  valueStyle={{ 
                    color: performance.errorCount > 0 ? '#ff4d4f' : '#52c41a',
                    fontSize: '16px'
                  }}
                />
              </Col>
            </Row>
            <div style={{ marginTop: 16 }}>
              <div style={{ marginBottom: 8 }}>
                <span>连接质量: </span>
                <Tag color={wsConnected ? 'green' : 'red'}>
                  {wsConnected ? '优秀' : '断开'}
                </Tag>
              </div>
              <Progress
                percent={wsConnected ? 95 : 0}
                status={wsConnected ? 'active' : 'exception'}
                strokeColor={wsConnected ? '#52c41a' : '#ff4d4f'}
              />
            </div>
          </Card>
        </Col>
        
        {/* 系统健康状态 */}
        <Col span={12}>
          <Card size="small" title="系统健康状态" bordered={false}>
            <Space direction="vertical" style={{ width: '100%' }}>
              <div>
                <span>整体状态: </span>
                <Tag color={wsConnected && connection.api && connection.ros ? 'green' : 'red'}>
                  {wsConnected && connection.api && connection.ros ? '健康' : '异常'}
                </Tag>
              </div>
              <div>
                <span>WebSocket: </span>
                <Tag color={getConnectionColor(wsConnected)}>
                  {getConnectionText(wsConnected)}
                </Tag>
              </div>
              <div>
                <span>后端API: </span>
                <Tag color={getConnectionColor(connection.api)}>
                  {getConnectionText(connection.api)}
                </Tag>
              </div>
              <div>
                <span>ROS系统: </span>
                <Tag color={getConnectionColor(connection.ros)}>
                  {getConnectionText(connection.ros)}
                </Tag>
              </div>
            </Space>
          </Card>
        </Col>
      </Row>
    </Card>
  );
};

export default ConnectionStatus;
