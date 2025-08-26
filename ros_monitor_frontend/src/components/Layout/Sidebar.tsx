import React from 'react';
import { Menu } from 'antd';
import {
  DashboardOutlined,
  CameraOutlined,
  WifiOutlined,
  SettingOutlined,
  VideoCameraOutlined
} from '@ant-design/icons';
import { useSystemStore } from '../../stores/useSystemStore';

export const Sidebar: React.FC = () => {
  const { ui, setCurrentPage } = useSystemStore();

  const menuItems = [
    {
      key: 'dashboard',
      icon: <DashboardOutlined />,
      label: '仪表盘',
    },
    {
      key: 'camera',
      icon: <CameraOutlined />,
      label: '相机监控',
    },
    {
      key: 'data-collection',
      icon: <VideoCameraOutlined />,
      label: '数据录制',
    },
    {
      key: 'connection',
      icon: <WifiOutlined />,
      label: '连接状态',
    },
    {
      key: 'settings',
      icon: <SettingOutlined />,
      label: '设置',
    },
  ];

  return (
    <div
      style={{
        position: 'fixed',
        left: 0,
        top: 0,
        bottom: 0,
        width: ui.sidebarCollapsed ? 80 : 240,
        background: '#001529',
        zIndex: 101,
        transition: 'width 0.2s ease',
        display: 'flex',
        flexDirection: 'column',
        boxShadow: '2px 0 8px rgba(0,0,0,0.15)',
      }}
    >
      {/* 标题区域 */}
      <div
        style={{
          height: '64px',
          display: 'flex',
          alignItems: 'center',
          justifyContent: 'center',
          color: 'white',
          fontSize: '20px',
          fontWeight: 'bold',
          borderBottom: '1px solid #303030',
          padding: '0 16px',
          textAlign: 'center',
        }}
      >
        {ui.sidebarCollapsed ? 'ROS' : 'ROS Monitor'}
      </div>
      
      {/* 菜单区域 */}
      <div style={{ flex: 1, overflow: 'hidden' }}>
        <Menu
          theme="dark"
          mode="inline"
          defaultSelectedKeys={[ui.currentPage]}
          selectedKeys={[ui.currentPage]}
          items={menuItems}
          onClick={({ key }) => setCurrentPage(key)}
          style={{
            border: 'none',
            background: 'transparent',
          }}
        />
      </div>
      
      {/* 底部描述 */}
      {!ui.sidebarCollapsed && (
        <div
          style={{
            padding: '16px',
            color: '#8c8c8c',
            fontSize: '12px',
            lineHeight: '1.4',
            textAlign: 'center',
            borderTop: '1px solid #303030',
          }}
        >
          <div>ROS系统监控仪表盘</div>
          <div>实时监控ROS系统状态、</div>
          <div>连接性能和传感器数据</div>
        </div>
      )}
    </div>
  );
};

