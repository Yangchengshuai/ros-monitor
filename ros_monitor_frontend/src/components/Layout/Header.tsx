import React from 'react';
import { Typography, Space, Badge, Tooltip } from 'antd';
import { 
  MenuFoldOutlined, 
  MenuUnfoldOutlined,
  WifiOutlined,
  ApiOutlined,
  RobotOutlined,
  ClockCircleOutlined
} from '@ant-design/icons';
import { useSystemStore } from '../../stores/useSystemStore';

const { Title } = Typography;

export const Header: React.FC = () => {
  const { connection, ui, toggleSidebar } = useSystemStore();

  const getStatusColor = (connected: boolean) => 
    connected ? '#52c41a' : '#ff4d4f';

  const getStatusText = (connected: boolean) =>
    connected ? '已连接' : '未连接';

  return (
    <div
      style={{
        position: 'sticky',
        top: 0,
        zIndex: 100,
        height: '64px',
        padding: '0 24px',
        background: '#001529',
        display: 'flex',
        alignItems: 'center',
        justifyContent: 'space-between',
        boxShadow: '0 2px 8px rgba(0,0,0,0.1)',
      }}
    >
      <Space align="center">
        <div
          onClick={toggleSidebar}
          style={{
            fontSize: '18px',
            color: 'white',
            cursor: 'pointer',
            padding: '4px',
          }}
        >
          {ui.sidebarCollapsed ? <MenuUnfoldOutlined /> : <MenuFoldOutlined />}
        </div>
        <Title level={4} style={{ margin: 0, color: 'white' }}>
          ROS远程监控系统
        </Title>
      </Space>

      <Space size="large">
        {/* WebSocket连接状态 */}
        <Tooltip title={`WebSocket: ${getStatusText(connection.websocket)}`}>
          <Badge 
            color={getStatusColor(connection.websocket)}
            dot
          >
            <WifiOutlined style={{ color: 'white', fontSize: '16px' }} />
          </Badge>
        </Tooltip>

        {/* API连接状态 */}
        <Tooltip title={`API: ${getStatusText(connection.api)}`}>
          <Badge 
            color={getStatusColor(connection.api)}
            dot
          >
            <ApiOutlined style={{ color: 'white', fontSize: '16px' }} />
          </Badge>
        </Tooltip>

        {/* ROS连接状态 */}
        <Tooltip title={`ROS: ${getStatusText(connection.ros)}`}>
          <Badge 
            color={getStatusColor(connection.ros)}
            dot
          >
            <RobotOutlined style={{ color: 'white', fontSize: '16px' }} />
          </Badge>
        </Tooltip>

        {/* 延迟显示 */}
        <Tooltip title={`延迟: ${connection.latency}ms`}>
          <Space style={{ color: 'white' }}>
            <ClockCircleOutlined />
            <span>{connection.latency}ms</span>
          </Space>
        </Tooltip>
      </Space>
    </div>
  );
};

