# ROS监控系统快速启动指南

## 🚀 一键启动（推荐）

```bash
# 在IKing Handbot根目录下运行
chmod +x start_monitor_system.sh
./start_monitor_system.sh
```

## 📋 手动启动步骤

### 步骤1: 准备环境

```bash
# 1. 加载ROS环境
source /opt/ros/noetic/setup.bash

# 2. 加载IKing Handbot工作空间
cd /home/ycs/work/ikinghandbot
source devel/setup.bash

# 3. 检查ROS Master是否运行
rostopic list
```

### 步骤2: 启动后端服务

```bash
cd ros_monitor_backend

# 激活虚拟环境
source .venv/bin/activate

# 检查环境
python check_environment.py

# 启动服务
./start_backend.sh
```

### 步骤3: 启动前端服务

```bash
# 新开一个终端
cd ros_monitor_frontend

# 启动前端
./start_frontend.sh
```

## 🌐 访问地址

- **前端界面**: http://localhost:5173
- **后端API**: http://localhost:8000
- **API文档**: http://localhost:8000/docs
- **健康检查**: http://localhost:8000/api/v1/health

## 🔧 环境要求

### 系统要求
- Ubuntu 20.04+ (推荐)
- Python 3.8+
- Node.js 16+
- ROS Noetic

### 依赖检查
```bash
# 检查Python依赖
cd ros_monitor_backend
source .venv/bin/activate
python check_environment.py

# 检查前端依赖
cd ../ros_monitor_frontend
npm list
```

## 🐛 常见问题

### 1. 虚拟环境问题
```bash
# 重新创建虚拟环境
cd ros_monitor_backend
rm -rf .venv
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

### 2. ROS环境问题
```bash
# 检查ROS安装
echo $ROS_DISTRO
which roscore

# 重新加载ROS环境
source /opt/ros/noetic/setup.bash
```

### 3. 端口占用问题
```bash
# 检查端口占用
lsof -i :8000
lsof -i :5173

# 停止占用进程
pkill -f "uvicorn.*8000"
pkill -f "vite.*5173"
```

### 4. 权限问题
```bash
# 给脚本添加执行权限
chmod +x ros_monitor_backend/start_backend.sh
chmod +x ros_monitor_frontend/start_frontend.sh
chmod +x start_monitor_system.sh
```

## 📱 使用说明

### 1. 相机监控
- 打开前端界面，选择"相机监控"
- 点击相机开关开启数据流
- 使用设置按钮调整图像参数

### 2. 系统状态
- 查看"仪表盘"了解系统状态
- 监控WebSocket连接状态
- 检查ROS节点运行情况

### 3. 连接状态
- 查看"连接状态"页面
- 监控前后端连接情况
- 检查数据推送状态

## 🛑 停止系统

```bash
# 方法1: 使用PID停止
pkill -f "ros_monitor"

# 方法2: 停止特定服务
pkill -f "uvicorn.*8000"  # 后端
pkill -f "vite.*5173"     # 前端

# 方法3: 在启动脚本中按Enter键
```

## 📞 技术支持

如果遇到问题，请：

1. 运行环境检查脚本：`python check_environment.py`
2. 查看日志输出
3. 检查ROS话题状态：`rostopic list`
4. 确认网络端口可用性

## 🔄 更新系统

```bash
# 更新后端
cd ros_monitor_backend
source .venv/bin/activate
git pull
pip install -r requirements.txt

# 更新前端
cd ../ros_monitor_frontend
git pull
npm install
```