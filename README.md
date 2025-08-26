# ROS Monitor - 多传感器监控系统

## 项目简介

ROS Monitor 是一个基于 FastAPI 和 React 的多传感器监控平台，专为机器人系统设计，支持激光雷达、IMU、相机等传感器数据的实时监控和可视化。

## 功能特性

- 🚀 **实时监控**: 支持多种ROS话题的实时数据监控
- 📊 **数据可视化**: 基于ECharts的数据图表展示
- 📷 **相机监控**: 支持压缩图像和原始图像的实时显示
- 🔌 **WebSocket通信**: 实时双向通信，低延迟数据推送
- 🎯 **传感器管理**: 统一的传感器状态监控和控制面板
- 📱 **响应式设计**: 支持桌面和移动设备的现代化UI

## 系统架构

```
ROS Monitor
├── Frontend (React + TypeScript + Vite)
│   ├── 实时数据展示
│   ├── 传感器控制面板
│   ├── 数据图表可视化
│   └── 响应式Web界面
├── Backend (FastAPI + Python)
│   ├── ROS话题订阅
│   ├── WebSocket服务
│   ├── RESTful API
│   └── 数据预处理
└── ROS Bridge
    ├── 传感器驱动
    ├── 话题管理
    └── 数据同步
```

## 技术栈

### 前端
- **React 19** - 现代化UI框架
- **TypeScript** - 类型安全的JavaScript
- **Vite** - 快速构建工具
- **Ant Design** - 企业级UI组件库
- **ECharts** - 数据可视化图表库
- **Zustand** - 轻量级状态管理

### 后端
- **FastAPI** - 高性能Python Web框架
- **Uvicorn** - ASGI服务器
- **WebSockets** - 实时通信
- **OpenCV** - 图像处理
- **NumPy** - 数值计算

### 系统集成
- **ROS (Robot Operating System)** - 机器人操作系统
- **cv_bridge** - ROS图像转换
- **rospy** - ROS Python客户端

## 快速开始

### 环境要求

- Python 3.8+
- Node.js 18+
- ROS Noetic/Melodic
- Ubuntu 18.04/20.04

### 安装依赖

#### 后端依赖
```bash
cd ros_monitor_backend
pip install -r requirements.txt
```

#### 前端依赖
```bash
cd ros_monitor_frontend
npm install
```

### 启动系统

#### 启动后端服务
```bash
cd ros_monitor_backend
./start_backend.sh
```

#### 启动前端服务
```bash
cd ros_monitor_frontend
npm run dev
```

#### 一键启动所有服务
```bash
./start_monitor_system.sh
```

### 访问系统

- 前端界面: http://localhost:5173
- 后端API: http://localhost:8000
- API文档: http://localhost:8000/docs

## 配置说明

### 传感器配置

在 `config/` 目录下配置各种传感器的参数：

- 激光雷达配置
- IMU参数设置
- 相机参数配置
- 话题名称映射

### 网络配置

- WebSocket端口: 8001
- HTTP API端口: 8000
- 前端开发端口: 5173

## 开发指南

### 项目结构

```
ROS_monitor/
├── ros_monitor_backend/          # 后端服务
│   ├── src/                     # 源代码
│   ├── tests/                   # 测试文件
│   ├── requirements.txt         # Python依赖
│   └── start_backend.sh        # 启动脚本
├── ros_monitor_frontend/        # 前端应用
│   ├── src/                     # React组件
│   ├── public/                  # 静态资源
│   ├── package.json            # Node.js依赖
│   └── start_frontend.sh       # 启动脚本
├── docs/                        # 项目文档
├── scripts/                     # 系统脚本
└── Documents/                   # 技术文档
```

### 添加新传感器

1. 在 `ros_monitor_backend/src/ros_bridge/subscribers/` 下创建新的订阅器
2. 在 `ros_monitor_frontend/src/components/Sensors/` 下添加对应的UI组件
3. 更新WebSocket消息类型和API接口

### 测试

```bash
# 后端测试
cd ros_monitor_backend
python -m pytest tests/

# 前端测试
cd ros_monitor_frontend
npm run test
```

## 部署

### Docker部署

```bash
# 构建镜像
docker build -t ros-monitor .

# 运行容器
docker run -p 8000:8000 -p 5173:5173 ros-monitor
```

### 生产环境

- 使用Nginx作为反向代理
- 配置SSL证书
- 设置环境变量
- 监控和日志管理

## 贡献指南

1. Fork 本仓库
2. 创建特性分支 (`git checkout -b feature/AmazingFeature`)
3. 提交更改 (`git commit -m 'Add some AmazingFeature'`)
4. 推送到分支 (`git push origin feature/AmazingFeature`)
5. 创建 Pull Request

## 许可证

本项目采用 MIT 许可证 - 查看 [LICENSE](LICENSE) 文件了解详情

## 联系方式

- 项目维护者: Yangchengshuai
- GitHub: [@Yangchengshuai](https://github.com/Yangchengshuai)
- 项目地址: [ros-monitor](https://github.com/Yangchengshuai/ros-monitor)

## 更新日志

### v1.0.0 (2025-08-26)
- 🎉 初始版本发布
- ✨ 支持激光雷达、IMU、相机监控
- 🚀 实时WebSocket通信
- 📊 数据可视化图表
- 🎨 现代化React UI界面

---

⭐ 如果这个项目对您有帮助，请给我们一个星标！