# ROS监控系统测试指南

## 概述

本文档介绍如何测试ROS监控系统的WebSocket服务和相机远程监控功能。

## 系统架构

```
ROS节点 (相机数据) → ROS桥接服务 → WebSocket服务器 → 前端界面
```

## 测试前准备

### 1. 安装依赖

```bash
# 后端依赖
pip install fastapi uvicorn websockets aiohttp opencv-python

# 前端依赖
cd ../ros_monitor_frontend
npm install
```

### 2. 启动ROS环境

```bash
# 启动ROS Master (如果还没有运行)
roscore

# 或者启动现有的ROS系统
source /home/ycs/work/ikinghandbot/devel/setup.bash
```

## 测试步骤

### 步骤1: 启动后端服务

```bash
cd /home/ycs/work/ikinghandbot/ros_monitor_backend

# 启动后端服务
python -m src.main
```

预期输出:
```
INFO - Starting ROS Monitor Backend...
INFO - ROS Node Manager initialized successfully
INFO - ROS subscribers setup completed. Total: X
INFO - Uvicorn running on http://0.0.0.0:8000
```

### 步骤2: 测试HTTP API

```bash
# 健康检查
curl http://localhost:8000/api/v1/health

# 系统状态
curl http://localhost:8000/api/v1/system/status
```

预期响应:
```json
{
  "success": true,
  "message": "ok",
  "ros_ready": true,
  "timestamp": 1234567890.123,
  "websocket_clients": 0
}
```

### 步骤3: 测试WebSocket连接

```bash
# 运行WebSocket测试脚本
python test_websocket_integration.py
```

预期输出:
```
INFO - 🚀 开始WebSocket集成测试
INFO - === 测试基本连接 ===
INFO - WebSocket connected with client ID: test_client_1234567890
INFO - ✅ 基本连接测试通过
INFO - === 测试话题订阅 ===
INFO - ✅ 话题订阅测试通过
INFO - === 测试系统状态请求 ===
INFO - ✅ 系统状态请求测试通过
INFO - 🎉 所有测试完成
```

### 步骤4: 运行完整系统测试

```bash
# 运行系统集成测试
python test_system.py
```

### 步骤5: 启动前端界面

```bash
cd ../ros_monitor_frontend

# 启动开发服务器
npm run dev
```

在浏览器中访问: `http://localhost:5173`

## 测试用例

### 1. 基本连接测试
- ✅ WebSocket连接建立
- ✅ 连接确认消息接收
- ✅ 客户端ID分配

### 2. 话题订阅测试
- ✅ 订阅相机话题
- ✅ 订阅确认消息
- ✅ 取消订阅功能

### 3. 数据推送测试
- ✅ 相机数据实时推送
- ✅ 数据格式正确性
- ✅ 时间戳准确性

### 4. 前端显示测试
- ✅ 相机图像显示
- ✅ 实时数据更新
- ✅ 控制功能正常

## 故障排除

### 常见问题

#### 1. ROS节点初始化失败
```
ERROR - Failed to initialize ROS Node Manager: ...
```
**解决方案:**
- 检查ROS环境是否正确加载
- 确认ROS Master是否运行
- 检查相机话题是否存在

#### 2. WebSocket连接失败
```
ERROR - Failed to connect: ...
```
**解决方案:**
- 确认后端服务是否启动
- 检查端口8000是否被占用
- 查看防火墙设置

#### 3. 相机数据不显示
**解决方案:**
- 检查相机话题是否有数据发布
- 确认前端是否正确订阅话题
- 查看浏览器控制台错误信息

#### 4. 图像显示异常
**解决方案:**
- 检查Base64编码是否正确
- 确认图像格式是否支持
- 查看Canvas绘制错误

### 调试技巧

#### 1. 启用详细日志
```python
# 在main.py中设置
logging.basicConfig(level=logging.DEBUG)
```

#### 2. 检查WebSocket消息
```javascript
// 在浏览器控制台中
ws.onmessage = (event) => {
    console.log('Received:', JSON.parse(event.data));
};
```

#### 3. 监控ROS话题
```bash
# 检查相机话题
rostopic list | grep camera

# 查看话题数据
rostopic echo /left_camera/image
```

## 性能测试

### 1. 连接数测试
```bash
# 模拟多个客户端连接
for i in {1..10}; do
    python test_websocket_integration.py &
done
```

### 2. 数据吞吐量测试
- 监控CPU和内存使用
- 检查网络带宽占用
- 测试图像压缩效果

### 3. 延迟测试
- 测量端到端延迟
- 检查帧率稳定性
- 评估用户体验

## 测试报告

### 成功标准
- [ ] 后端服务正常启动
- [ ] HTTP API响应正常
- [ ] WebSocket连接建立
- [ ] 相机数据实时推送
- [ ] 前端界面正常显示
- [ ] 控制功能正常工作

### 测试环境
- **操作系统**: Ubuntu 20.04
- **ROS版本**: Noetic
- **Python版本**: 3.8+
- **Node.js版本**: 16+
- **浏览器**: Chrome 90+

### 测试结果
- **测试时间**: [填写]
- **测试人员**: [填写]
- **通过率**: [填写]
- **问题记录**: [填写]

## 后续改进

### 1. 自动化测试
- 集成CI/CD流程
- 自动化测试脚本
- 性能基准测试

### 2. 监控告警
- 系统健康检查
- 性能指标监控
- 异常告警机制

### 3. 文档完善
- API文档生成
- 用户操作手册
- 故障处理指南