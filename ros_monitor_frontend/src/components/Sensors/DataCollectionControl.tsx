import React, { useState, useEffect } from 'react';
import { Button, Card, Row, Col, message } from 'antd';
import { PlayCircleOutlined, StopOutlined, ReloadOutlined } from '@ant-design/icons';
import { dataCollectionService } from '../../services/dataCollection';

// 定义数据采集状态接口
interface DataCollectionStatus {
  is_running: boolean;
  process_id: number | null;
  start_time: number | null;
  script_path: string;
  last_update: number;
}

interface DataCollectionControlProps {
  className?: string;
}

export const DataCollectionControl: React.FC<DataCollectionControlProps> = ({ className }) => {
  const [status, setStatus] = useState<DataCollectionStatus>({
    is_running: false,
    process_id: null,
    start_time: null,
    script_path: '',
    last_update: 0
  });
  const [loading, setLoading] = useState(false);

  // 获取当前状态
  const fetchStatus = async () => {
    try {
      const response = await dataCollectionService.getStatus();
      if (response.success && response.data) {
        setStatus(response.data);
      }
    } catch (error) {
      console.error('Failed to fetch status:', error);
    }
  };

  // 启动采集
  const handleStart = async () => {
    setLoading(true);
    try {
      const response = await dataCollectionService.startCollection();
      if (response.success) {
        message.success(response.message || '数据采集已启动');
        await fetchStatus();
      } else {
        message.error(response.message || '启动失败');
      }
    } catch (error) {
      message.error('启动请求失败');
    } finally {
      setLoading(false);
    }
  };

  // 停止采集
  const handleStop = async () => {
    setLoading(true);
    try {
      const response = await dataCollectionService.stopCollection();
      if (response.success) {
        message.success(response.message || '数据采集已停止');
        await fetchStatus();
      } else {
        message.error(response.message || '停止失败');
      }
    } catch (error) {
      message.error('停止请求失败');
    } finally {
      setLoading(false);
    }
  };

  // 组件挂载时获取状态
  useEffect(() => {
    fetchStatus();
    
    // 每5秒刷新一次状态
    const interval = setInterval(fetchStatus, 5000);
    return () => clearInterval(interval);
  }, []);

  // 计算运行时长
  const getDuration = () => {
    if (!status.is_running || !status.start_time) return '';
    const duration = Math.floor((Date.now() / 1000) - status.start_time);
    const hours = Math.floor(duration / 3600);
    const minutes = Math.floor((duration % 3600) / 60);
    const seconds = duration % 60;
    return `${hours.toString().padStart(2, '0')}:${minutes.toString().padStart(2, '0')}:${seconds.toString().padStart(2, '0')}`;
  };

  return (
    <Card
      title="数据采集控制"
      className={className}
      style={{ marginBottom: 16 }}
    >
      <Row gutter={16} align="middle">
        <Col span={8}>
          <Button
            type="primary"
            icon={<PlayCircleOutlined />}
            onClick={handleStart}
            disabled={status.is_running || loading}
            loading={loading && !status.is_running}
            style={{ width: '100%' }}
          >
            开始采集
          </Button>
        </Col>
        
        <Col span={8}>
          <Button
            type="default"
            icon={<StopOutlined />}
            onClick={handleStop}
            disabled={!status.is_running || loading}
            loading={loading && status.is_running}
            style={{ width: '100%' }}
          >
            停止采集
          </Button>
        </Col>
        
        <Col span={8}>
          <Button
            icon={<ReloadOutlined />}
            onClick={fetchStatus}
            loading={loading}
            style={{ width: '100%' }}
          >
            刷新状态
          </Button>
        </Col>
      </Row>

      <div style={{ marginTop: 16, textAlign: 'center' }}>
        <div>
          状态: <strong style={{ 
            color: status.is_running ? '#52c41a' : '#d9d9d9' 
          }}>
            {status.is_running ? '采集中' : '已停止'}
          </strong>
        </div>
        
        {status.is_running && (
          <div>
            运行时长: <strong>{getDuration()}</strong>
          </div>
        )}
      </div>
    </Card>
  );
};

export default DataCollectionControl;