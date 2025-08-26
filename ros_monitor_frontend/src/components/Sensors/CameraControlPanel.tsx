import React, { useState } from 'react';
import { Card, Row, Col, Button, Switch, Select, InputNumber, Form, message } from 'antd';
import { 
  PlayCircleOutlined, 
  PauseCircleOutlined, 
  ReloadOutlined,
  SettingOutlined,
  CameraOutlined
} from '@ant-design/icons';
import DataCollectionControl from './DataCollectionControl';

const { Option } = Select;

interface CameraControlPanelProps {
  onStartStreaming: () => void;
  onStopStreaming: () => void;
  onRefresh: () => void;
  onSettingsChange: (settings: any) => void;
}

export const CameraControlPanel: React.FC<CameraControlPanelProps> = ({
  onStartStreaming,
  onStopStreaming,
  onRefresh,
  onSettingsChange
}) => {
  const [isStreaming, setIsStreaming] = useState(false);
  const [form] = Form.useForm();

  const handleStreamToggle = (checked: boolean) => {
    setIsStreaming(checked);
    if (checked) {
      onStartStreaming();
      message.success('相机流已启动');
    } else {
      onStopStreaming();
      message.info('相机流已停止');
    }
  };

  const handleRefresh = () => {
    onRefresh();
    message.info('正在刷新相机数据...');
  };

  const handleSettingsSubmit = (values: any) => {
    onSettingsChange(values);
    message.success('相机设置已更新');
  };

  return (
    <Card
      title={
        <div style={{ display: 'flex', alignItems: 'center', gap: 8 }}>
          <CameraOutlined />
          <span>相机控制面板</span>
        </div>
      }
      style={{ marginBottom: 16 }}
    >
      <Row gutter={16} style={{ marginBottom: 16 }}>
        <Col span={6}>
          <Button
            type="primary"
            icon={isStreaming ? <PauseCircleOutlined /> : <PlayCircleOutlined />}
            onClick={() => handleStreamToggle(!isStreaming)}
            style={{ width: '100%' }}
          >
            {isStreaming ? '停止流' : '启动流'}
          </Button>
        </Col>
        <Col span={6}>
          <Button
            icon={<ReloadOutlined />}
            onClick={handleRefresh}
            style={{ width: '100%' }}
          >
            刷新数据
          </Button>
        </Col>
        <Col span={6}>
          <Switch
            checked={isStreaming}
            onChange={handleStreamToggle}
            checkedChildren="流媒体开启"
            unCheckedChildren="流媒体关闭"
          />
        </Col>
        <Col span={6}>
          <Button
            icon={<SettingOutlined />}
            style={{ width: '100%' }}
            onClick={() => form.submit()}
          >
            应用设置
          </Button>
        </Col>
      </Row>

      <Form
        form={form}
        layout="inline"
        onFinish={handleSettingsSubmit}
        initialValues={{
          resolution: '640x480',
          frameRate: 30,
          quality: 80,
          compression: 'jpeg'
        }}
      >
        <Row gutter={16} style={{ width: '100%' }}>
          <Col span={6}>
            <Form.Item label="分辨率" name="resolution">
              <Select style={{ width: '100%' }}>
                <Option value="320x240">320x240</Option>
                <Option value="640x480">640x480</Option>
                <Option value="1280x720">1280x720</Option>
                <Option value="1920x1080">1920x1080</Option>
              </Select>
            </Form.Item>
          </Col>
          <Col span={6}>
            <Form.Item label="帧率" name="frameRate">
              <InputNumber
                min={1}
                max={60}
                style={{ width: '100%' }}
                addonAfter="fps"
              />
            </Form.Item>
          </Col>
          <Col span={6}>
            <Form.Item label="质量" name="quality">
              <InputNumber
                min={10}
                max={100}
                style={{ width: '100%' }}
                addonAfter="%"
              />
            </Form.Item>
          </Col>
          <Col span={6}>
            <Form.Item label="压缩" name="compression">
              <Select style={{ width: '100%' }}>
                <Option value="jpeg">JPEG</Option>
                <Option value="png">PNG</Option>
                <Option value="webp">WebP</Option>
              </Select>
            </Form.Item>
          </Col>
        </Row>
      </Form>
      
      {/* 数据采集控制组件 */}
      <DataCollectionControl />
    </Card>
  );
};

export default CameraControlPanel;