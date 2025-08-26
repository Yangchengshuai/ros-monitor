import React, { useEffect } from 'react';

import { Header } from './Header';
import { Sidebar } from './Sidebar';
import { SystemStatus } from '../Dashboard/SystemStatus';
import { ConnectionStatus } from '../Dashboard/ConnectionStatus';
import CameraMonitor from '../CameraMonitor';
import DataCollectionControl from '../Sensors/DataCollectionControl';
import { useSystemStore } from '../../stores/useSystemStore';
import { apiService } from '../../services/api';

export const MainLayout: React.FC = () => {
  const { ui } = useSystemStore();
  const currentPage = ui.currentPage;

  // 初始化API健康检查
  useEffect(() => {
    console.log('🚀 启动API健康检查...');
    
    // 开始定期健康检查（每10秒检查一次）
    apiService.startHealthCheck(10000);
    
    // 组件卸载时停止健康检查
    return () => {
      console.log('🛑 停止API健康检查...');
      apiService.stopHealthCheck();
    };
  }, []);

  const renderContent = () => {
    switch (currentPage) {
      case 'dashboard':
        return <SystemStatus />;
      case 'camera':
        return <CameraMonitor />;
      case 'connection':
        return <ConnectionStatus />;
      case 'data-collection':
        return <DataCollectionControl />;
      case 'settings':
        return (
          <div style={{ padding: '24px', textAlign: 'center' }}>
            <h2>系统设置</h2>
            <p>设置功能开发中...</p>
          </div>
        );
      default:
        return <SystemStatus />;
    }
  };

  return (
    <div style={{ 
      display: 'flex', 
      minHeight: '100vh',
      backgroundColor: '#f0f2f5'
    }}>
      {/* 侧边栏 */}
      <Sidebar />
      
      {/* 主内容区域 */}
      <div style={{ 
        flex: 1,
        display: 'flex',
        flexDirection: 'column',
        marginLeft: ui.sidebarCollapsed ? 80 : 240,
        transition: 'margin-left 0.2s ease',
        minWidth: 0, // 防止flex子元素溢出
      }}>
        {/* 顶部导航 */}
        <Header />
        
        {/* 主内容 */}
        <div style={{
          flex: 1,
          padding: '24px',
          marginTop: '64px', // Header高度
          overflow: 'auto',
          minHeight: 'calc(100vh - 64px)',
        }}>
          <div style={{
            background: '#fff',
            borderRadius: '8px',
            padding: '24px',
            boxShadow: '0 2px 8px rgba(0,0,0,0.1)',
            minHeight: 'calc(100vh - 112px)', // 减去Header和padding
          }}>
            {renderContent()}
          </div>
        </div>
      </div>
    </div>
  );
};

