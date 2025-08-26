# ROS监控系统 (ROS Monitor System)

一个基于FastAPI和React的多传感器融合监控平台，专为IKing Handbot机器人项目设计，支持激光雷达、IMU、相机等传感器数据的实时监控和可视化。

## 🚀 项目特性

- **实时数据监控**: 支持ROS话题的实时订阅和数据流
- **多传感器支持**: 激光雷达、IMU、相机等多种传感器
- **WebSocket通信**: 实时数据推送和双向通信
- **现代化前端**: 基于React + TypeScript + Ant Design
- **RESTful API**: 完整的后端API接口
- **系统监控**: 算法启停控制、录制控制、系统状态监控

## 🏗️ 系统架构

```
ROS Monitor System
├── ros_monitor_backend/     # FastAPI后端服务
│   ├── src/                # 核心源代码
│   ├── tests/              # 测试文件
│   ├── scripts/            # 启动和部署脚本
│   └── requirements.txt    # Python依赖
├── ros_monitor_frontend/   # React前端应用
│   ├── src/                # 前端源代码
│   ├── public/             # 静态资源
│   └── package.json        # Node.js依赖
├── script/                 # 系统级脚本
├── Documents/              # 项目文档
└── docs/                   # 技术文档
```

## 🛠️ 技术栈

### 后端
- **FastAPI**: 现代Python Web框架
- **WebSocket**: 实时双向通信
- **ROS集成**: rospy, cv_bridge等
- **数据处理**: OpenCV, NumPy

### 前端
- **React 19**: 最新版本React框架
- **TypeScript**: 类型安全的JavaScript
- **Ant Design**: 企业级UI组件库
- **ECharts**: 数据可视化图表
- **Zustand**: 轻量级状态管理

## 📦 快速开始

### 环境要求
- Python 3.8+
- Node.js 18+
- ROS Noetic/Melodic
- Ubuntu 20.04+

### 1. 克隆项目
```bash
git clone <repository-url>
cd ros-monitor-system
```

### 2. 启动后端服务
```bash
cd ros_monitor_backend

# 创建虚拟环境
python3 -m venv .venv
source .venv/bin/activate

# 安装依赖
pip install -r requirements.txt

# 启动服务
./start_backend.sh
```

### 3. 启动前端应用
```bash
cd ros_monitor_frontend

# 安装依赖
npm install

# 开发模式
npm run dev

# 生产构建
npm run build
```

### 4. 一键启动系统
```bash
# 在项目根目录
./start_monitor_system.sh
```

## 🔧 配置说明

### 后端配置
- 默认端口: 8000
- WebSocket端点: `/ws`
- API文档: `/docs`

### 前端配置
- 开发端口: 5173
- 构建输出: `dist/`目录

## 📊 功能模块

### 传感器监控
- **激光雷达**: 点云数据实时显示
- **IMU**: 姿态和运动数据监控
- **相机**: 图像流和视频监控

### 系统控制
- **算法启停**: 控制SLAM算法运行
- **录制控制**: 数据录制管理
- **状态监控**: 系统运行状态

### 数据可视化
- **实时图表**: ECharts图表展示
- **3D可视化**: 点云和轨迹显示
- **历史数据**: 数据回放和分析

## 🧪 测试

### 后端测试
```bash
cd ros_monitor_backend
python -m pytest tests/
```

### 前端测试
```bash
cd ros_monitor_frontend
npm run lint
npm test
```

### 端到端测试
```bash
python test_end_to_end.py
```

## 📚 文档

- [快速启动指南](ros_monitor_backend/README_QuickStart.md)
- [测试指南](ros_monitor_backend/README_TESTING.md)
- [相机调试指南](ros_monitor_backend/README_CAMERA_DEBUG.md)
- [API文档](http://localhost:8000/docs)

## 🤝 贡献指南

1. Fork项目
2. 创建功能分支 (`git checkout -b feature/AmazingFeature`)
3. 提交更改 (`git commit -m 'Add some AmazingFeature'`)
4. 推送到分支 (`git push origin feature/AmazingFeature`)
5. 创建Pull Request

## 📄 许可证

本项目采用MIT许可证 - 查看 [LICENSE](LICENSE) 文件了解详情

## 🙏 致谢

- IKing Handbot项目团队
- ROS社区
- FastAPI和React开源社区

## 📞 联系方式

- 项目维护者: YCS
- 项目地址: [GitHub Repository]
- 问题反馈: [Issues]

---

**注意**: 本项目需要ROS环境支持，请确保已正确安装和配置ROS系统。
