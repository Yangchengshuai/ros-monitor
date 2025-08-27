# ROS Monitor Frontend

ROS远程监控系统前端，基于React + TypeScript + Vite构建，支持实时监控相机、激光雷达、IMU等传感器数据。

## 功能特性

- 🎥 实时相机数据流监控
- 📡 WebSocket实时数据传输
- 🌐 支持局域网远程访问
- 📱 响应式设计，支持移动端
- ⚡ 高性能图像渲染
- 🔧 可配置的相机参数

## 快速开始

### 环境要求

- Node.js 16+
- npm 或 yarn

### 安装依赖

```bash
npm install
```

### 开发模式

```bash
npm run dev
```

访问 http://localhost:3000

### 构建生产版本

```bash
npm run build
```

## 局域网访问配置

### 问题描述

局域网其他电脑访问时，相机无法打开开关，无法查看相机数据。

### 解决方案

1. **配置环境变量**

在项目根目录创建 `.env.local` 文件：

```bash
# 局域网访问配置
VITE_API_HOST=192.168.1.100
VITE_LAN_IP=192.168.1.100
VITE_API_PORT=8000
VITE_WS_PORT=8000
```

**重要**：将 `192.168.1.100` 替换为你的服务器实际局域网IP地址。

2. **网络配置检查**

确保：
- 服务器防火墙允许8000端口访问
- 局域网内其他电脑能够ping通服务器IP
- 后端服务绑定到 `0.0.0.0:8000`

3. **使用方法**

```bash
# 配置环境变量后重启服务
npm run dev

# 局域网内其他电脑访问
# http://192.168.1.100:3000
```

详细配置说明请参考 [LAN_ACCESS_CONFIG.md](./LAN_ACCESS_CONFIG.md)

## 项目结构

```
src/
├── components/          # React组件
│   ├── CameraMonitor/  # 相机监控组件
│   ├── CameraViewer/   # 相机视图组件
│   └── Layout/         # 布局组件
├── services/           # 服务层
│   ├── api.ts         # API服务
│   ├── websocket.ts   # WebSocket服务
│   └── dataCollection.ts # 数据采集服务
├── stores/             # 状态管理
├── hooks/              # 自定义Hook
├── utils/              # 工具函数
└── types/              # 类型定义
```

## 技术栈

- **前端框架**: React 18 + TypeScript
- **构建工具**: Vite
- **UI组件库**: Ant Design
- **状态管理**: Zustand
- **WebSocket**: 原生WebSocket API
- **样式**: CSS + Ant Design

## 开发规范

- 使用TypeScript进行类型检查
- 遵循ESLint规则
- 组件使用函数式组件 + Hooks
- 状态管理使用Zustand
- 网络请求使用fetch API

## 故障排除

### 常见问题

1. **WebSocket连接失败**
   - 检查服务器IP地址配置
   - 确认防火墙设置
   - 验证后端服务状态

2. **相机数据不显示**
   - 检查WebSocket连接状态
   - 确认订阅请求是否发送成功
   - 查看浏览器控制台错误信息

3. **局域网访问问题**
   - 检查环境变量配置
   - 确认服务器网络配置
   - 验证端口是否对外开放

## 贡献指南

1. Fork 项目
2. 创建功能分支 (`git checkout -b feature/AmazingFeature`)
3. 提交更改 (`git commit -m 'Add some AmazingFeature'`)
4. 推送到分支 (`git push origin feature/AmazingFeature`)
5. 打开 Pull Request

## 许可证

本项目采用 MIT 许可证 - 查看 [LICENSE](../LICENSE) 文件了解详情
