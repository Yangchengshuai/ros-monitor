# ROS Monitor Backend 快速启动指南

## 当前状态

✅ **已完成**：基础框架代码、WebSocket通信、ROS桥接  
📋 **待测试**：ROS环境集成、传感器数据流  

## 快速启动步骤

### 1. 环境准备

```bash
cd /home/ycs/work/ikinghandbot/ros_monitor_backend

# 创建Python虚拟环境
python3 -m venv .venv
source .venv/bin/activate

# 安装依赖
pip install -r requirements.txt

# 确保ROS环境
source /opt/ros/noetic/setup.bash
source /home/ycs/work/ikinghandbot/devel/setup.bash
```

### 2. 环境测试

```bash
# 运行环境测试脚本
python3 test_setup.py
```

预期输出应该显示所有✅标记。如果有❌，按提示修复。

### 3. 启动服务

```bash
# 方式1：直接启动
./scripts/start.sh

# 方式2：开发模式（带reload）
export PYTHONPATH=/home/ycs/work/ikinghandbot/ros_monitor_backend
uvicorn src.main:app --host 0.0.0.0 --port 8000 --reload
```

### 4. 功能验证

#### 4.1 健康检查
```bash
# 检查API
curl http://localhost:8000/api/v1/health

# 预期响应
{"success": true, "message": "ok", "ros_ready": false}
```

#### 4.2 WebSocket连接测试
浏览器访问：http://localhost:8000/docs  
使用FastAPI自动生成的WebSocket测试界面。

#### 4.3 ROS话题测试
```bash
# 启动ROS Master（如果未运行）
roscore &

# 发布测试数据
rostopic pub /test_topic std_msgs/String "data: 'Hello ROS Monitor'"

# 检查话题列表
rostopic list
```

## 下一步开发建议

### A. 当前可以开发测试的功能：
1. ✅ **HTTP API测试** - 健康检查接口已可用
2. ✅ **WebSocket连接** - 基础连接和消息传递
3. ⚠️ **ROS集成测试** - 需要ROS节点运行

### B. 需要优先验证：
1. **ROS话题订阅** - 启动 `/livox/lidar`、`/livox/imu`、相机话题
2. **数据格式转换** - 确认ROS消息→JSON转换
3. **WebSocket数据推送** - 实时数据流测试

### C. 建议开发顺序：

#### 第1步：基础功能验证（本周）
- [ ] 环境搭建和依赖安装
- [ ] 基础API和WebSocket连接测试
- [ ] ROS话题订阅功能验证

#### 第2步：传感器集成（下周）
- [ ] 相机数据流集成
- [ ] IMU数据集成  
- [ ] 点云数据集成（采样+压缩）

#### 第3步：控制功能（第3周）
- [ ] 算法启停控制API
- [ ] 录制控制功能
- [ ] 系统状态监控

#### 第4步：优化完善（第4周）
- [ ] 性能优化和错误处理
- [ ] WebRTC视频流集成
- [ ] 前端界面开发

## 故障排除

### 问题1：rospy导入失败
```bash
# 解决方案
source /opt/ros/noetic/setup.bash
export PYTHONPATH=$PYTHONPATH:/opt/ros/noetic/lib/python3/dist-packages
```

### 问题2：cv_bridge导入失败
```bash
# 安装ROS OpenCV桥接
sudo apt install ros-noetic-cv-bridge
```

### 问题3：端口占用
```bash
# 检查端口占用
lsof -i :8000
# 释放端口或更改配置中的端口
```

## 生产部署（systemd）

```bash
# 复制service文件
sudo cp /home/ycs/work/ikinghandbot/Documents/systemd/ros-monitor-backend.service /etc/systemd/system/

# 启用服务
sudo systemctl enable ros-monitor-backend
sudo systemctl start ros-monitor-backend

# 检查状态
sudo systemctl status ros-monitor-backend
```

---

**总结**：当前系统已具备运行基础，建议先进行环境测试，然后逐步验证各模块功能。

