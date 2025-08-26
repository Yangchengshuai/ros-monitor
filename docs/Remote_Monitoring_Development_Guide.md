# ROS远程监控系统开发实现指南

## 概述

基于**HTTP API + WebSocket混合方案**的ROS远程监控系统开发文档，提供完整的代码架构、API设计、实现示例和开发规范。

**项目目标**：为IKing Handbot多传感器融合系统提供Web端远程监控和控制能力。

---

## 技术架构

### 系统架构图

```
┌─────────────────────────────────────────────────────────────────┐
│                    NX设备 (Ubuntu 20.04)                        │
├─────────────────────────────────────────────────────────────────┤
│  ROS Master (localhost:11311)                                  │
│  ├── /livox/lidar        (CustomMsg)                          │
│  ├── /livox/imu          (sensor_msgs/Imu)                    │
│  ├── /left_camera/image  (sensor_msgs/Image)                  │
│  └── /right_camera/image (sensor_msgs/Image)                  │
├─────────────────────────────────────────────────────────────────┤
│  监控服务层                                                      │
│  ├── FastAPI Backend     (Port: 8000)                         │
│  │   ├── HTTP API Server  (RESTful控制接口)                   │
│  │   ├── WebSocket Server (实时数据推送)                      │
│  │   └── ROS Bridge       (ROS话题订阅器)                     │
│  ├── MJPEG Stream Server (Port: 8080)                         │
│  └── Static File Server  (Port: 3000)                         │
└─────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────┐
│                     远程客户端                                   │
├─────────────────────────────────────────────────────────────────┤
│  Web浏览器 (Chrome/Firefox/Edge)                               │
│  ├── React Frontend App                                        │
│  │   ├── 实时相机流显示                                        │
│  │   ├── 3D点云可视化                                         │
│  │   ├── 系统状态监控                                         │
│  │   └── 控制面板                                             │
│  ├── HTTP Client        (axios)                               │
│  ├── WebSocket Client   (实时数据接收)                         │
│  └── MJPEG Decoder      (相机图像流)                          │
└─────────────────────────────────────────────────────────────────┘
```

### 生产基线与SLA

- 相机视频(生产): WebRTC(H.264/NVENC) 640x360@5fps，端到端延迟<300ms；带宽约0.5–1.5 Mbps/路
- 相机视频(调试/缩略图): 320x240@≤1fps，经WebSocket二进制/Base64，仅用于诊断缩略
- 点云: 关键帧/ROI/体素下采样(5 cm)，≤1 Hz，单帧≤100k点；端到端<800ms；带宽≤3 Mbps
- IMU/状态: JSON/MessagePack，10–50 Hz，端到端<200ms
- 并发: 默认≤10 Web客户端；超出触发降级策略
- 稳定性: 可用性≥99%；异常自动降级与自愈；故障恢复<60s

以上为默认保守SLA，可按网络与NX资源在配置中调整；所有阈值需可观测并可热更新。

### 技术栈选择

#### 后端技术栈
- **Web框架**: FastAPI 0.104+ (高性能异步框架)
- **WebSocket**: FastAPI内置WebSocket支持
- **ROS接口**: rospy 1.16+ (ROS Python客户端)
- **图像处理**: OpenCV 4.5+ (图像编码和处理)
- **数据序列化**: JSON + MessagePack (高效二进制序列化)
- **认证**: JWT Token + bcrypt (安全认证)
- **配置管理**: Pydantic Settings (类型安全配置)
- **日志**: structlog (结构化日志)

#### 前端技术栈  
- **框架**: React 18+ with TypeScript
- **状态管理**: Zustand (轻量级状态管理)
- **UI组件**: Ant Design 5+ (企业级UI库)
- **3D可视化**: Three.js + React-Three-Fiber (WebGL渲染)
- **图表**: ECharts (数据可视化)
- **HTTP客户端**: Axios (API调用)
- **WebSocket**: native WebSocket API
- **构建工具**: Vite (快速构建)

---

## 项目结构设计

### 后端项目结构

```
ros_monitor_backend/
├── pyproject.toml              # 项目依赖配置
├── requirements.txt            # Python依赖
├── Dockerfile                  # Docker构建文件
├── docker-compose.yml          # 容器编排
├── .env.example               # 环境变量模板
├── README.md                  # 项目说明
├── src/
│   ├── main.py                # FastAPI应用入口
│   ├── config/                # 配置管理
│   │   ├── __init__.py
│   │   ├── settings.py        # 应用配置
│   │   └── ros_config.py      # ROS配置
│   ├── api/                   # HTTP API路由
│   │   ├── __init__.py
│   │   ├── v1/                # API版本管理
│   │   │   ├── __init__.py
│   │   │   ├── control.py     # 系统控制API
│   │   │   ├── recording.py   # 录制控制API
│   │   │   ├── system.py      # 系统状态API
│   │   │   └── auth.py        # 认证API
│   │   └── deps.py            # 依赖注入
│   ├── websocket/             # WebSocket处理
│   │   ├── __init__.py
│   │   ├── connection_manager.py # 连接管理器
│   │   ├── handlers/          # 消息处理器
│   │   │   ├── __init__.py
│   │   │   ├── camera_handler.py    # 相机数据处理
│   │   │   ├── lidar_handler.py     # 点云数据处理
│   │   │   └── system_handler.py    # 系统消息处理
│   │   └── schemas.py         # WebSocket消息模式
│   ├── ros_bridge/            # ROS桥接层
│   │   ├── __init__.py
│   │   ├── node_manager.py    # ROS节点管理
│   │   ├── subscribers/       # 话题订阅器
│   │   │   ├── __init__.py
│   │   │   ├── camera_subscriber.py  # 相机订阅器
│   │   │   ├── lidar_subscriber.py   # 雷达订阅器
│   │   │   └── imu_subscriber.py     # IMU订阅器
│   │   ├── publishers/        # 话题发布器
│   │   │   ├── __init__.py
│   │   │   └── control_publisher.py # 控制命令发布
│   │   └── services/          # ROS服务客户端
│   │       ├── __init__.py
│   │       └── system_service.py    # 系统服务调用
│   ├── services/              # 业务逻辑服务
│   │   ├── __init__.py
│   │   ├── auth_service.py    # 认证服务
│   │   ├── recording_service.py # 录制服务
│   │   ├── system_service.py  # 系统服务
│   │   └── streaming_service.py # 流媒体服务
│   ├── models/                # 数据模型
│   │   ├── __init__.py
│   │   ├── user.py           # 用户模型
│   │   ├── system.py         # 系统状态模型
│   │   └── sensor.py         # 传感器数据模型
│   ├── utils/                 # 工具模块
│   │   ├── __init__.py
│   │   ├── compression.py    # 数据压缩
│   │   ├── security.py       # 安全工具
│   │   ├── logger.py         # 日志配置
│   │   └── validators.py     # 数据验证
│   └── tests/                 # 单元测试
│       ├── __init__.py
│       ├── test_api/         # API测试
│       ├── test_websocket/   # WebSocket测试
│       └── test_ros_bridge/  # ROS桥接测试
└── scripts/                   # 脚本工具
    ├── setup.sh              # 环境搭建脚本
    ├── start.sh              # 启动脚本
    └── deploy.sh             # 部署脚本
```

### 前端项目结构

```
ros_monitor_frontend/
├── package.json               # NPM依赖配置
├── tsconfig.json             # TypeScript配置
├── vite.config.ts            # Vite构建配置
├── tailwind.config.js        # Tailwind CSS配置
├── Dockerfile                # Docker构建文件
├── .env.example              # 环境变量模板
├── README.md                 # 项目说明
├── public/                   # 静态资源
│   ├── index.html
│   ├── favicon.ico
│   └── robots.txt
├── src/
│   ├── main.tsx              # 应用入口
│   ├── App.tsx               # 主应用组件
│   ├── types/                # TypeScript类型定义
│   │   ├── api.ts           # API接口类型
│   │   ├── websocket.ts     # WebSocket消息类型
│   │   └── sensors.ts       # 传感器数据类型
│   ├── components/           # React组件
│   │   ├── common/          # 通用组件
│   │   │   ├── Loading.tsx
│   │   │   ├── ErrorBoundary.tsx
│   │   │   └── Layout.tsx
│   │   ├── camera/          # 相机组件
│   │   │   ├── CameraViewer.tsx     # 相机查看器
│   │   │   ├── CameraControls.tsx   # 相机控制
│   │   │   └── CameraSettings.tsx   # 相机设置
│   │   ├── lidar/           # 雷达组件
│   │   │   ├── LidarViewer.tsx      # 点云查看器  
│   │   │   ├── LidarControls.tsx    # 雷达控制
│   │   │   └── PointCloudCanvas.tsx # 3D渲染画布
│   │   ├── control/         # 控制组件
│   │   │   ├── ControlPanel.tsx     # 主控制面板
│   │   │   ├── SystemControls.tsx   # 系统控制
│   │   │   └── RecordingControls.tsx # 录制控制
│   │   ├── monitor/         # 监控组件
│   │   │   ├── SystemStatus.tsx     # 系统状态
│   │   │   ├── SensorStatus.tsx     # 传感器状态
│   │   │   └── PerformanceMonitor.tsx # 性能监控
│   │   └── auth/            # 认证组件
│   │       ├── LoginForm.tsx
│   │       └── UserProfile.tsx
│   ├── services/            # 服务层
│   │   ├── api.ts          # HTTP API封装
│   │   ├── websocket.ts    # WebSocket服务
│   │   ├── auth.ts         # 认证服务
│   │   └── streaming.ts    # 流媒体服务
│   ├── stores/             # 状态管理
│   │   ├── useSystemStore.ts    # 系统状态
│   │   ├── useSensorStore.ts    # 传感器状态
│   │   ├── useAuthStore.ts      # 认证状态
│   │   └── useUIStore.ts        # UI状态
│   ├── hooks/              # 自定义Hooks
│   │   ├── useWebSocket.ts     # WebSocket Hook
│   │   ├── useCamera.ts        # 相机Hook
│   │   ├── useLidar.ts         # 雷达Hook
│   │   └── useAuth.ts          # 认证Hook
│   ├── utils/              # 工具函数
│   │   ├── compression.ts      # 数据解压缩
│   │   ├── validation.ts       # 数据验证
│   │   ├── formatters.ts       # 数据格式化
│   │   └── constants.ts        # 常量定义
│   └── styles/             # 样式文件
│       ├── globals.css         # 全局样式
│       ├── components.css      # 组件样式
│       └── theme.ts            # 主题配置
├── docs/                   # 文档
│   ├── DEVELOPMENT.md      # 开发文档
│   ├── API.md             # API文档
│   └── DEPLOYMENT.md      # 部署文档
└── scripts/                # 构建脚本
    ├── build.sh           # 构建脚本
    ├── deploy.sh          # 部署脚本
    └── test.sh            # 测试脚本
```

---

## API接口设计

### HTTP API规范

#### 基础配置
- **Base URL**: `http://<NX_IP>:8000/api/v1`
- **认证方式**: Bearer Token (JWT)
- **数据格式**: JSON
- **响应格式**: 统一响应包装

#### 通用响应结构

```typescript
interface ApiResponse<T = any> {
  success: boolean;
  message: string;
  data?: T;
  error?: string;
  timestamp: string;
}

interface PaginatedResponse<T = any> extends ApiResponse<T[]> {
  pagination: {
    page: number;
    limit: number;
    total: number;
    totalPages: number;
  };
}
```

#### 1. 认证接口

```typescript
// POST /api/v1/auth/login
interface LoginRequest {
  username: string;
  password: string;
}

interface LoginResponse {
  access_token: string;
  refresh_token: string;
  token_type: "bearer";
  expires_in: number;
  user: {
    id: string;
    username: string;
    role: string;
  };
}

// POST /api/v1/auth/refresh  
interface RefreshRequest {
  refresh_token: string;
}

// POST /api/v1/auth/logout
// DELETE /api/v1/auth/sessions/{session_id}
```

#### 2. 系统控制接口

```typescript
// GET /api/v1/system/status
interface SystemStatus {
  ros_master: {
    connected: boolean;
    uri: string;
    topics: number;
    nodes: number;
  };
  sensors: {
    camera: SensorStatus;
    lidar: SensorStatus;
    imu: SensorStatus;
  };
  algorithms: {
    fast_livo: AlgorithmStatus;
  };
  resources: {
    cpu_usage: number;
    memory_usage: number;
    disk_usage: number;
    network_io: NetworkIO;
  };
  uptime: number;
}

interface SensorStatus {
  active: boolean;
  frequency: number;
  last_message: string;
  error_count: number;
}

interface AlgorithmStatus {
  running: boolean;
  status: "idle" | "running" | "paused" | "error";
  pid?: number;
  start_time?: string;
  error_message?: string;
}

// POST /api/v1/system/control
interface SystemControlRequest {
  action: "start_all" | "stop_all" | "restart_all";
  components?: string[];  // 可选择性控制特定组件
}

// POST /api/v1/system/algorithms/fast-livo/control
interface AlgorithmControlRequest {
  action: "start" | "stop" | "pause" | "resume";
  config?: {
    launch_file?: string;
    parameters?: Record<string, any>;
  };
}
```

#### 3. 录制控制接口

```typescript
// GET /api/v1/recording/status
interface RecordingStatus {
  active: boolean;
  current_bag?: {
    filename: string;
    size: number;
    duration: number;
    topics: string[];
  };
  history: RecordingHistory[];
}

interface RecordingHistory {
  filename: string;
  start_time: string;
  end_time: string;
  size: number;
  duration: number;
  topics: string[];
}

// POST /api/v1/recording/start
interface StartRecordingRequest {
  bag_name?: string;
  topics?: string[];
  max_duration?: number;  // 最大录制时长(秒)
  max_size?: number;      // 最大文件大小(MB)
}

// POST /api/v1/recording/stop
// GET /api/v1/recording/files
// DELETE /api/v1/recording/files/{filename}
// GET /api/v1/recording/files/{filename}/download
```

#### 4. 传感器控制接口

```typescript
// GET /api/v1/sensors/camera/info
interface CameraInfo {
  cameras: CameraDevice[];
}

interface CameraDevice {
  id: string;
  name: string;
  serial_number: string;
  status: SensorStatus;
  settings: CameraSettings;
}

interface CameraSettings {
  resolution: [number, number];
  frame_rate: number;
  exposure: number;
  gain: number;
  trigger_mode: boolean;
}

// POST /api/v1/sensors/camera/{camera_id}/settings
interface UpdateCameraSettingsRequest {
  settings: Partial<CameraSettings>;
}

// GET /api/v1/sensors/lidar/info
interface LidarInfo {
  device_info: LidarDevice;
  point_cloud_info: PointCloudInfo;
}

interface PointCloudInfo {
  points_per_second: number;
  frequency: number;
  range_max: number;
  field_of_view: number;
}

// GET /api/v1/sensors/imu/info
interface ImuInfo {
  device_info: ImuDevice;
  calibration: ImuCalibration;
}
```

---

## WebSocket通信协议

### 连接管理

```typescript
// WebSocket连接地址
const WS_URL = `ws://${HOST}:8000/ws/{client_id}`;

// 连接认证
interface WSAuthMessage {
  type: "auth";
  token: string;
}

// 连接确认
interface WSConnectedMessage {
  type: "connected";
  client_id: string;
  server_time: string;
}
```

### 消息格式规范

```typescript
// 基础消息结构
interface WSMessage {
  type: string;
  timestamp: string;
  data: any;
}

// 订阅消息
interface WSSubscribeMessage extends WSMessage {
  type: "subscribe";
  data: {
    topics: string[];
    frequency?: number;  // 期望频率
    compression?: boolean; // 是否压缩
  };
}

// 取消订阅
interface WSUnsubscribeMessage extends WSMessage {
  type: "unsubscribe";
  data: {
    topics: string[];
  };
}
```

### 传感器数据推送

#### 1. 相机图像数据（生产建议：WebRTC 推流，WebSocket仅限缩略/调试）

```typescript
interface WSCameraMessage extends WSMessage {
  type: "camera_image";
  data: {
    camera_id: "left" | "right";
    timestamp: string;
    sequence: number;
    encoding: "jpeg" | "png"; // 禁止生产上通过WS传raw大图
    width: number;
    height: number;
    data: string;  // Base64或二进制，仅限低帧率缩略/诊断
    compressed: boolean;
  };
}
```

#### 2. 点云数据

```typescript
interface WSLidarMessage extends WSMessage {
  type: "lidar_pointcloud";
  data: {
    timestamp: string;
    frame_id: string;
    point_count: number;
    fields: PointField[];
    data: number[];  // 压缩后的点云数据
    compression: "none" | "gzip" | "lz4";
  };
}

interface PointField {
  name: string;
  offset: number;
  datatype: number;
  count: number;
}
```

#### 3. IMU数据

```typescript
interface WSImuMessage extends WSMessage {
  type: "imu_data";
  data: {
    timestamp: string;
    orientation: {
      x: number;
      y: number;
      z: number;
      w: number;
    };
    angular_velocity: {
      x: number;
      y: number;
      z: number;
    };
    linear_acceleration: {
      x: number;
      y: number;
      z: number;
    };
  };
}
```

#### 4. 系统状态推送

```typescript
interface WSSystemStatusMessage extends WSMessage {
  type: "system_status";
  data: {
    ros_nodes: NodeStatus[];
    cpu_usage: number;
    memory_usage: number;
    network_io: NetworkIO;
    errors: SystemError[];
  };
}

interface NodeStatus {
  name: string;
  status: "running" | "stopped" | "error";
  pid?: number;
  cpu_usage: number;
  memory_usage: number;
}

interface SystemError {
  level: "error" | "warning" | "info";
  message: string;
  timestamp: string;
  source: string;
}
```

---

## 核心代码实现示例

### 后端核心代码

#### 1. FastAPI主应用 (`src/main.py`)

```python
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
import uvicorn
import asyncio
from contextlib import asynccontextmanager

from src.config.settings import get_settings
from src.api.v1 import control, recording, system, auth
from src.websocket.connection_manager import ConnectionManager
from src.ros_bridge.node_manager import ROSNodeManager
from src.utils.logger import setup_logger

logger = setup_logger(__name__)
settings = get_settings()

# ROS节点管理器和WebSocket连接管理器
ros_manager = None
connection_manager = ConnectionManager()

@asynccontextmanager
async def lifespan(app: FastAPI):
    """应用生命周期管理"""
    global ros_manager
    
    # 启动时初始化ROS
    logger.info("Starting ROS Monitor Backend...")
    ros_manager = ROSNodeManager()
    await ros_manager.initialize()
    
    # 启动后台任务
    asyncio.create_task(data_streaming_task())
    
    yield
    
    # 关闭时清理
    logger.info("Shutting down ROS Monitor Backend...")
    if ros_manager:
        await ros_manager.shutdown()

# 创建FastAPI应用
app = FastAPI(
    title="ROS Remote Monitor API",
    description="远程ROS系统监控和控制API",
    version="1.0.0",
    lifespan=lifespan
)

# 中间件配置
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.CORS_ORIGINS,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# API路由注册
app.include_router(auth.router, prefix="/api/v1/auth", tags=["authentication"])
app.include_router(system.router, prefix="/api/v1/system", tags=["system"])
app.include_router(control.router, prefix="/api/v1/control", tags=["control"])
app.include_router(recording.router, prefix="/api/v1/recording", tags=["recording"])

# WebSocket端点
@app.websocket("/ws/{client_id}")
async def websocket_endpoint(websocket: WebSocket, client_id: str):
    await connection_manager.connect(websocket, client_id)
    try:
        while True:
            # 接收客户端消息
            message = await websocket.receive_json()
            await handle_websocket_message(client_id, message)
    except WebSocketDisconnect:
        connection_manager.disconnect(client_id)
        logger.info(f"Client {client_id} disconnected")

async def handle_websocket_message(client_id: str, message: dict):
    """处理WebSocket消息"""
    msg_type = message.get("type")
    
    if msg_type == "subscribe":
        topics = message.get("data", {}).get("topics", [])
        await connection_manager.subscribe_topics(client_id, topics)
    elif msg_type == "unsubscribe":
        topics = message.get("data", {}).get("topics", [])
        await connection_manager.unsubscribe_topics(client_id, topics)
    else:
        logger.warning(f"Unknown message type: {msg_type}")

async def data_streaming_task():
    """后台数据流推送任务"""
    while True:
        try:
            # 检查ROS数据并推送给订阅的客户端
            if ros_manager and ros_manager.is_connected():
                # 获取最新的传感器数据
                camera_data = await ros_manager.get_latest_camera_data()
                lidar_data = await ros_manager.get_latest_lidar_data()
                imu_data = await ros_manager.get_latest_imu_data()
                
                # 推送给订阅的客户端
                if camera_data:
                    await connection_manager.broadcast_to_subscribers("camera", camera_data)
                if lidar_data:
                    await connection_manager.broadcast_to_subscribers("lidar", lidar_data)
                if imu_data:
                    await connection_manager.broadcast_to_subscribers("imu", imu_data)
                    
        except Exception as e:
            logger.error(f"Data streaming error: {e}")
        
        await asyncio.sleep(0.1)  # 10Hz推送频率

if __name__ == "__main__":
    uvicorn.run(
        "main:app",
        host=settings.HOST,
        port=settings.PORT,
        reload=settings.DEBUG,
        log_level="info"
    )
```

#### 2. ROS桥接节点管理器 (`src/ros_bridge/node_manager.py`)

```python
import asyncio
import rospy
from threading import Thread
from typing import Dict, Any, Optional, List
import json

from src.ros_bridge.subscribers.camera_subscriber import CameraSubscriber
from src.ros_bridge.subscribers.lidar_subscriber import LidarSubscriber
from src.ros_bridge.subscribers.imu_subscriber import ImuSubscriber
from src.utils.logger import setup_logger

logger = setup_logger(__name__)

class ROSNodeManager:
    """ROS节点管理器，负责管理所有ROS相关的订阅和发布"""
    
    def __init__(self):
        self.ros_thread: Optional[Thread] = None
        self.subscribers: Dict[str, Any] = {}
        self.latest_data: Dict[str, Any] = {}
        self._running = False
        
    async def initialize(self):
        """初始化ROS节点"""
        try:
            # 在单独线程中初始化ROS
            self.ros_thread = Thread(target=self._init_ros_node, daemon=True)
            self.ros_thread.start()
            
            # 等待ROS初始化完成
            await asyncio.sleep(2.0)
            
            if not rospy.is_shutdown():
                self._setup_subscribers()
                self._running = True
                logger.info("ROS Node Manager initialized successfully")
            else:
                raise Exception("Failed to initialize ROS node")
                
        except Exception as e:
            logger.error(f"ROS initialization failed: {e}")
            raise
            
    def _init_ros_node(self):
        """在单独线程中初始化ROS节点"""
        try:
            rospy.init_node('ros_monitor_bridge', anonymous=True, disable_signals=True)
            rospy.spin()  # 保持ROS节点运行
        except Exception as e:
            logger.error(f"ROS node initialization failed: {e}")
            
    def _setup_subscribers(self):
        """设置ROS话题订阅器"""
        try:
            # 相机订阅器
            self.subscribers['left_camera'] = CameraSubscriber(
                topic='/left_camera/image',
                callback=lambda data: self._update_data('left_camera', data)
            )
            self.subscribers['right_camera'] = CameraSubscriber(
                topic='/right_camera/image', 
                callback=lambda data: self._update_data('right_camera', data)
            )
            
            # 激光雷达订阅器
            self.subscribers['lidar'] = LidarSubscriber(
                topic='/livox/lidar',
                callback=lambda data: self._update_data('lidar', data)
            )
            
            # IMU订阅器
            self.subscribers['imu'] = ImuSubscriber(
                topic='/livox/imu',
                callback=lambda data: self._update_data('imu', data)
            )
            
            logger.info("ROS subscribers setup completed")
            
        except Exception as e:
            logger.error(f"Failed to setup ROS subscribers: {e}")
            
    def _update_data(self, topic: str, data: Dict[str, Any]):
        """更新最新数据"""
        self.latest_data[topic] = {
            'timestamp': rospy.get_time(),
            'data': data
        }
        
    async def get_latest_camera_data(self) -> Optional[Dict[str, Any]]:
        """获取最新相机数据"""
        data = {}
        for camera in ['left_camera', 'right_camera']:
            if camera in self.latest_data:
                data[camera] = self.latest_data[camera]
        return data if data else None
        
    async def get_latest_lidar_data(self) -> Optional[Dict[str, Any]]:
        """获取最新激光雷达数据"""
        return self.latest_data.get('lidar')
        
    async def get_latest_imu_data(self) -> Optional[Dict[str, Any]]:
        """获取最新IMU数据"""
        return self.latest_data.get('imu')
        
    def is_connected(self) -> bool:
        """检查ROS连接状态"""
        return self._running and not rospy.is_shutdown()
        
    async def shutdown(self):
        """关闭ROS节点管理器"""
        try:
            self._running = False
            if not rospy.is_shutdown():
                rospy.signal_shutdown("Application shutdown")
            logger.info("ROS Node Manager shutdown completed")
        except Exception as e:
            logger.error(f"ROS shutdown error: {e}")
```

#### 3. 相机订阅器 (`src/ros_bridge/subscribers/camera_subscriber.py`)

```python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import base64
from typing import Callable, Dict, Any

from src.utils.compression import compress_image
from src.utils.logger import setup_logger

logger = setup_logger(__name__)

class CameraSubscriber:
    """相机数据订阅器"""
    
    def __init__(self, topic: str, callback: Callable[[Dict[str, Any]], None]):
        self.topic = topic
        self.callback = callback
        self.bridge = CvBridge()
        self.subscriber = None
        self.frame_count = 0
        
        self._setup_subscriber()
        
    def _setup_subscriber(self):
        """设置ROS订阅器"""
        try:
            self.subscriber = rospy.Subscriber(
                self.topic,
                Image,
                self._image_callback,
                queue_size=1
            )
            logger.info(f"Camera subscriber for {self.topic} created")
        except Exception as e:
            logger.error(f"Failed to create camera subscriber for {self.topic}: {e}")
            
    def _image_callback(self, msg: Image):
        """图像消息回调函数"""
        try:
            # 转换ROS图像消息到OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 压缩图像
            encoded_image, compressed_size = compress_image(cv_image, quality=80)
            
            # 构造数据包
            camera_data = {
                'camera_id': self.topic.split('/')[-2],  # 从topic提取相机ID
                'timestamp': msg.header.stamp.to_sec(),
                'sequence': self.frame_count,
                'encoding': 'jpeg',
                'width': msg.width,
                'height': msg.height,
                'data': base64.b64encode(encoded_image).decode('utf-8'),
                'compressed': True,
                'original_size': msg.width * msg.height * 3,
                'compressed_size': compressed_size
            }
            
            # 调用回调函数
            self.callback(camera_data)
            self.frame_count += 1
            
        except Exception as e:
            logger.error(f"Camera callback error for {self.topic}: {e}")
```

### 前端核心代码

#### 1. WebSocket服务 (`src/services/websocket.ts`)

```typescript
import { useSystemStore } from '../stores/useSystemStore';
import { useSensorStore } from '../stores/useSensorStore';

export interface WSMessage {
  type: string;
  timestamp: string;
  data: any;
}

export class WebSocketService {
  private ws: WebSocket | null = null;
  private reconnectAttempts = 0;
  private maxReconnectAttempts = 5;
  private reconnectInterval = 3000;
  private clientId: string;
  
  constructor(clientId?: string) {
    this.clientId = clientId || this.generateClientId();
  }
  
  private generateClientId(): string {
    return `client_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
  }
  
  connect(host: string, port: number = 8000): Promise<void> {
    return new Promise((resolve, reject) => {
      try {
        const wsUrl = `ws://${host}:${port}/ws/${this.clientId}`;
        this.ws = new WebSocket(wsUrl);
        
        this.ws.onopen = () => {
          console.log('WebSocket connected');
          this.reconnectAttempts = 0;
          
          // 发送认证消息
          const token = localStorage.getItem('access_token');
          if (token) {
            this.send({
              type: 'auth',
              timestamp: new Date().toISOString(),
              data: { token }
            });
          }
          
          resolve();
        };
        
        this.ws.onmessage = (event) => {
          try {
            const message: WSMessage = JSON.parse(event.data);
            this.handleMessage(message);
          } catch (error) {
            console.error('Failed to parse WebSocket message:', error);
          }
        };
        
        this.ws.onclose = () => {
          console.log('WebSocket disconnected');
          this.attemptReconnect();
        };
        
        this.ws.onerror = (error) => {
          console.error('WebSocket error:', error);
          reject(error);
        };
        
      } catch (error) {
        reject(error);
      }
    });
  }
  
  private handleMessage(message: WSMessage): void {
    const { type, data } = message;
    
    switch (type) {
      case 'connected':
        console.log('WebSocket authentication successful');
        break;
        
      case 'camera_image':
        this.handleCameraData(data);
        break;
        
      case 'lidar_pointcloud':
        this.handleLidarData(data);
        break;
        
      case 'imu_data':
        this.handleImuData(data);
        break;
        
      case 'system_status':
        this.handleSystemStatus(data);
        break;
        
      default:
        console.warn('Unknown message type:', type);
    }
  }
  
  private handleCameraData(data: any): void {
    const sensorStore = useSensorStore.getState();
    sensorStore.updateCameraData(data.camera_id, {
      timestamp: data.timestamp,
      imageData: data.data,
      width: data.width,
      height: data.height,
      encoding: data.encoding
    });
  }
  
  private handleLidarData(data: any): void {
    const sensorStore = useSensorStore.getState();
    sensorStore.updateLidarData({
      timestamp: data.timestamp,
      pointCount: data.point_count,
      points: data.data,
      fields: data.fields
    });
  }
  
  private handleImuData(data: any): void {
    const sensorStore = useSensorStore.getState();
    sensorStore.updateImuData({
      timestamp: data.timestamp,
      orientation: data.orientation,
      angularVelocity: data.angular_velocity,
      linearAcceleration: data.linear_acceleration
    });
  }
  
  private handleSystemStatus(data: any): void {
    const systemStore = useSystemStore.getState();
    systemStore.updateSystemStatus({
      rosNodes: data.ros_nodes,
      cpuUsage: data.cpu_usage,
      memoryUsage: data.memory_usage,
      networkIO: data.network_io,
      errors: data.errors
    });
  }
  
  send(message: WSMessage): void {
    if (this.ws && this.ws.readyState === WebSocket.OPEN) {
      this.ws.send(JSON.stringify(message));
    } else {
      console.error('WebSocket is not connected');
    }
  }
  
  subscribe(topics: string[]): void {
    this.send({
      type: 'subscribe',
      timestamp: new Date().toISOString(),
      data: { topics }
    });
  }
  
  unsubscribe(topics: string[]): void {
    this.send({
      type: 'unsubscribe', 
      timestamp: new Date().toISOString(),
      data: { topics }
    });
  }
  
  private attemptReconnect(): void {
    if (this.reconnectAttempts < this.maxReconnectAttempts) {
      this.reconnectAttempts++;
      console.log(`Attempting to reconnect... (${this.reconnectAttempts}/${this.maxReconnectAttempts})`);
      
      setTimeout(() => {
        const host = import.meta.env.VITE_API_HOST || 'localhost';
        const port = import.meta.env.VITE_WS_PORT || 8000;
        this.connect(host, port).catch(console.error);
      }, this.reconnectInterval);
    }
  }
  
  disconnect(): void {
    if (this.ws) {
      this.ws.close();
      this.ws = null;
    }
  }
}

// 单例WebSocket服务
export const wsService = new WebSocketService();
```

#### 2. 相机查看器组件 (`src/components/camera/CameraViewer.tsx`)

```tsx
import React, { useEffect, useRef, useState } from 'react';
import { Card, Select, Switch, Slider, Button } from 'antd';
import { FullscreenOutlined, DownloadOutlined, SettingOutlined } from '@ant-design/icons';
import { useSensorStore } from '../../stores/useSensorStore';
import { wsService } from '../../services/websocket';

const { Option } = Select;

interface CameraViewerProps {
  cameraId: 'left_camera' | 'right_camera';
  className?: string;
}

export const CameraViewer: React.FC<CameraViewerProps> = ({ 
  cameraId, 
  className 
}) => {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const [isStreaming, setIsStreaming] = useState(false);
  const [imageScale, setImageScale] = useState(1.0);
  const [showSettings, setShowSettings] = useState(false);
  
  // 从store获取相机数据
  const cameraData = useSensorStore(state => state.cameras[cameraId]);
  const updateCameraSettings = useSensorStore(state => state.updateCameraSettings);
  
  useEffect(() => {
    if (isStreaming) {
      // 订阅相机数据
      wsService.subscribe([cameraId]);
    } else {
      // 取消订阅
      wsService.unsubscribe([cameraId]);
    }
    
    return () => {
      wsService.unsubscribe([cameraId]);
    };
  }, [isStreaming, cameraId]);
  
  useEffect(() => {
    if (cameraData?.imageData && canvasRef.current) {
      drawImage();
    }
  }, [cameraData?.imageData, imageScale]);
  
  const drawImage = () => {
    const canvas = canvasRef.current;
    const ctx = canvas?.getContext('2d');
    
    if (!canvas || !ctx || !cameraData?.imageData) return;
    
    const img = new Image();
    img.onload = () => {
      // 调整canvas尺寸
      const scaledWidth = img.width * imageScale;
      const scaledHeight = img.height * imageScale;
      
      canvas.width = scaledWidth;
      canvas.height = scaledHeight;
      
      // 绘制图像
      ctx.clearRect(0, 0, scaledWidth, scaledHeight);
      ctx.drawImage(img, 0, 0, scaledWidth, scaledHeight);
      
      // 叠加信息
      drawOverlay(ctx, scaledWidth, scaledHeight);
    };
    
    img.src = `data:image/${cameraData.encoding};base64,${cameraData.imageData}`;
  };
  
  const drawOverlay = (ctx: CanvasRenderingContext2D, width: number, height: number) => {
    // 绘制时间戳和状态信息
    ctx.fillStyle = 'rgba(0, 0, 0, 0.7)';
    ctx.fillRect(10, 10, 300, 80);
    
    ctx.fillStyle = 'white';
    ctx.font = '14px Arial';
    ctx.fillText(`Camera: ${cameraId}`, 20, 30);
    ctx.fillText(`Timestamp: ${new Date(cameraData.timestamp * 1000).toLocaleTimeString()}`, 20, 50);
    ctx.fillText(`Resolution: ${cameraData.width}x${cameraData.height}`, 20, 70);
  };
  
  const handleStreamToggle = (checked: boolean) => {
    setIsStreaming(checked);
  };
  
  const handleScaleChange = (value: number) => {
    setImageScale(value);
  };
  
  const handleFullscreen = () => {
    const canvas = canvasRef.current;
    if (canvas && canvas.requestFullscreen) {
      canvas.requestFullscreen();
    }
  };
  
  const handleDownload = () => {
    const canvas = canvasRef.current;
    if (canvas) {
      const link = document.createElement('a');
      link.download = `${cameraId}_${Date.now()}.png`;
      link.href = canvas.toDataURL();
      link.click();
    }
  };
  
  const cameraTitle = cameraId === 'left_camera' ? '左相机' : '右相机';
  const connectionStatus = cameraData ? 'connected' : 'disconnected';
  
  return (
    <Card
      title={
        <div style={{ display: 'flex', alignItems: 'center', gap: 8 }}>
          <span>{cameraTitle}</span>
          <div 
            style={{ 
              width: 8, 
              height: 8, 
              borderRadius: '50%',
              backgroundColor: connectionStatus === 'connected' ? '#52c41a' : '#ff4d4f'
            }} 
          />
        </div>
      }
      extra={
        <div style={{ display: 'flex', gap: 8 }}>
          <Switch
            checked={isStreaming}
            onChange={handleStreamToggle}
            checkedChildren="ON"
            unCheckedChildren="OFF"
          />
          <Button 
            icon={<SettingOutlined />}
            onClick={() => setShowSettings(!showSettings)}
          />
          <Button 
            icon={<FullscreenOutlined />}
            onClick={handleFullscreen}
          />
          <Button 
            icon={<DownloadOutlined />}
            onClick={handleDownload}
          />
        </div>
      }
      className={className}
    >
      <div style={{ position: 'relative' }}>
        <canvas
          ref={canvasRef}
          style={{
            width: '100%',
            height: 'auto',
            border: '1px solid #d9d9d9',
            borderRadius: 4
          }}
        />
        
        {showSettings && (
          <div 
            style={{
              position: 'absolute',
              top: 10,
              right: 10,
              background: 'rgba(0, 0, 0, 0.8)',
              padding: 16,
              borderRadius: 4,
              color: 'white',
              minWidth: 200
            }}
          >
            <div style={{ marginBottom: 16 }}>
              <label>缩放比例: {imageScale.toFixed(1)}</label>
              <Slider
                min={0.1}
                max={2.0}
                step={0.1}
                value={imageScale}
                onChange={handleScaleChange}
              />
            </div>
          </div>
        )}
        
        {!isStreaming && (
          <div 
            style={{
              position: 'absolute',
              top: '50%',
              left: '50%',
              transform: 'translate(-50%, -50%)',
              textAlign: 'center',
              color: '#999'
            }}
          >
            <p>相机流已停止</p>
            <p>请开启开关以查看实时画面</p>
          </div>
        )}
      </div>
    </Card>
  );
};

export default CameraViewer;
```

---

## 开发环境搭建

### 后端环境搭建

```bash
# 1. 创建Python虚拟环境
cd /home/ycs/work/ikinghandbot
python3 -m venv venv_monitor
source venv_monitor/bin/activate

# 2. 安装依赖
pip install fastapi[all] uvicorn websockets
pip install opencv-python rospy cv_bridge
pip install pydantic pydantic-settings
pip install python-multipart python-jose[cryptography] passlib[bcrypt]
pip install structlog colorlog

# 3. 创建项目目录
mkdir -p ros_monitor_backend/src/{api/v1,websocket,ros_bridge,services,models,utils}

# 4. 创建配置文件
cat > ros_monitor_backend/.env << EOF
# 服务器配置
HOST=0.0.0.0
PORT=8000
DEBUG=true

# CORS配置
CORS_ORIGINS=["http://localhost:3000", "http://localhost:5173"]

# JWT配置
SECRET_KEY=your-secret-key-here
ALGORITHM=HS256
ACCESS_TOKEN_EXPIRE_MINUTES=30

# ROS配置
ROS_MASTER_URI=http://localhost:11311
ROS_HOSTNAME=localhost
EOF
```

### 前端环境搭建

```bash
# 1. 创建React项目
cd /home/ycs/work/ikinghandbot
npm create vite@latest ros_monitor_frontend -- --template react-ts
cd ros_monitor_frontend

# 2. 安装依赖
npm install antd @ant-design/icons
npm install axios zustand
npm install three @types/three @react-three/fiber @react-three/drei
npm install echarts echarts-for-react
npm install @types/node

# 3. 安装开发依赖
npm install -D tailwindcss postcss autoprefixer
npm install -D @types/react @types/react-dom

# 4. 初始化Tailwind CSS
npx tailwindcss init -p

# 5. 创建环境配置
cat > .env.local << EOF
VITE_API_HOST=localhost
VITE_API_PORT=8000
VITE_WS_PORT=8000
VITE_APP_TITLE=ROS远程监控系统
EOF
```

---

## 部署方案

### Docker容器化部署

#### 后端Dockerfile

```dockerfile
# ros_monitor_backend/Dockerfile
FROM python:3.9-slim

# 设置工作目录
WORKDIR /app

# 安装系统依赖
RUN apt-get update && apt-get install -y \
    build-essential \
    libopencv-dev \
    python3-opencv \
    && rm -rf /var/lib/apt/lists/*

# 复制依赖文件
COPY requirements.txt .

# 安装Python依赖
RUN pip install --no-cache-dir -r requirements.txt

# 复制源代码
COPY . .

# 暴露端口
EXPOSE 8000

# 启动命令
CMD ["uvicorn", "src.main:app", "--host", "0.0.0.0", "--port", "8000"]
```

#### 前端Dockerfile

```dockerfile
# ros_monitor_frontend/Dockerfile
FROM node:18-alpine AS builder

WORKDIR /app

# 复制package文件
COPY package*.json ./

# 安装依赖
RUN npm ci --only=production

# 复制源代码
COPY . .

# 构建应用
RUN npm run build

# 生产环境镜像
FROM nginx:alpine

# 复制构建结果
COPY --from=builder /app/dist /usr/share/nginx/html

# 复制nginx配置
COPY nginx.conf /etc/nginx/nginx.conf

EXPOSE 80

CMD ["nginx", "-g", "daemon off;"]
```

#### Docker Compose配置

```yaml
# docker-compose.yml
version: '3.8'

services:
  ros_monitor_backend:
    build: ./ros_monitor_backend
    container_name: ros_monitor_backend
    network_mode: host
    volumes:
      - /opt/ros:/opt/ros:ro
      - /home/ycs/work/data:/app/data
    environment:
      - ROS_MASTER_URI=http://localhost:11311
      - ROS_HOSTNAME=localhost
    depends_on:
      - redis
    restart: unless-stopped

  ros_monitor_frontend:
    build: ./ros_monitor_frontend
    container_name: ros_monitor_frontend
    ports:
      - "3000:80"
    environment:
      - VITE_API_HOST=${API_HOST:-localhost}
      - VITE_API_PORT=8000
    depends_on:
      - ros_monitor_backend
    restart: unless-stopped

  redis:
    image: redis:7-alpine
    container_name: ros_monitor_redis
    ports:
      - "6379:6379"
    volumes:
      - redis_data:/data
    restart: unless-stopped

volumes:
  redis_data:
```

### 生产环境部署脚本

```bash
#!/bin/bash
# scripts/deploy.sh

set -e

echo "=== ROS Monitor System Deployment ==="

# 检查环境
check_environment() {
    echo "Checking environment..."
    
    if ! command -v docker &> /dev/null; then
        echo "Error: Docker is not installed"
        exit 1
    fi
    
    if ! command -v docker-compose &> /dev/null; then
        echo "Error: Docker Compose is not installed"
        exit 1
    fi
    
    echo "✓ Environment check passed"
}

# 构建镜像
build_images() {
    echo "Building Docker images..."
    
    docker-compose build --no-cache
    
    echo "✓ Images built successfully"
}

# 启动服务
start_services() {
    echo "Starting services..."
    
    # 停止现有服务
    docker-compose down
    
    # 启动新服务
    docker-compose up -d
    
    echo "✓ Services started successfully"
}

# 健康检查
health_check() {
    echo "Performing health check..."
    
    # 等待服务启动
    sleep 30
    
    # 检查后端API
    if curl -f http://localhost:8000/api/v1/system/status > /dev/null 2>&1; then
        echo "✓ Backend API is healthy"
    else
        echo "✗ Backend API health check failed"
        exit 1
    fi
    
    # 检查前端
    if curl -f http://localhost:3000 > /dev/null 2>&1; then
        echo "✓ Frontend is healthy"
    else
        echo "✗ Frontend health check failed" 
        exit 1
    fi
}

# 主流程
main() {
    check_environment
    build_images
    start_services
    health_check
    
    echo ""
    echo "=== Deployment Completed Successfully ==="
    echo "Frontend: http://localhost:3000"
    echo "Backend API: http://localhost:8000"
    echo "API Documentation: http://localhost:8000/docs"
    echo ""
    echo "Use 'docker-compose logs -f' to view logs"
    echo "Use 'docker-compose down' to stop services"
}

main "$@"
```

### WebRTC 推流最小服务（GStreamer/NVENC）

#### 方案A：使用现成 webrtc-streamer（推荐快速落地）

- 适用：UVC摄像头（`/dev/video0` 等）；Jetson NX 可直接使用，硬件编码由内核/驱动决定。
- 启动（Docker）：
```bash
sudo docker run --rm --name webrtc-streamer \
  --network host \
  --device /dev/video0 \
  mpromonet/webrtc-streamer:latest \
  -H 0.0.0.0:8008 /dev/video0
```
- 访问：浏览器打开 `http://<NX_IP>:8008/`，选择 `/dev/video0` 进行播放。
- 自定义帧率/分辨率（UVC）：
```bash
v4l2-ctl -d /dev/video0 --set-fmt-video=width=640,height=360,pixelformat=MJPG
v4l2-ctl -d /dev/video0 --set-parm=15
```

#### 方案B：GStreamer + webrtcbin（需要信令服务）

- 适用：Jetson CSI 摄像头或自定义 Pipeline。
- 依赖：`gstreamer1.0-*`、`gstreamer1.0-plugins-bad`（含 webrtcbin）、Jetson 平台的 `nvv4l2h264enc`。
- 样例 Pipeline（CSI 摄像头 → H.264 → webrtcbin）：
```bash
gst-launch-1.0 -v \
  nvarguscamerasrc ! 'video/x-raw(memory:NVMM),width=640,height=360,framerate=15/1' ! \
  nvv4l2h264enc insert-sps-pps=true iframeinterval=15 bitrate=1000000 preset-level=1 ! \
  h264parse ! rtph264pay config-interval=-1 pt=96 ! \
  webrtcbin name=wb stun-server=stun://stun.l.google.com:19302
```
- 说明：`webrtcbin` 需要信令交换（SDP/ICE）。建议直接采用方案A的 webrtc-streamer 容器以简化信令。

#### 方案C：ROS图像 → WebRTC（后续扩展）

- 通过 Python/aiortc 或 GStreamer `appsrc` 将 `/left_camera/image` 转封装到 WebRTC；支持服务端限帧/限码率。
- 生产建议优先使用方案A，方案C作为 ROS 深度集成时再落地。

### Foxglove-bridge 集成与切换

- 安装（ROS 包）：`sudo apt install ros-noetic-foxglove-bridge`（或源码）
- 启动：
```bash
source /home/ycs/work/ikinghandbot/devel/setup.bash
rosrun foxglove_bridge foxglove_bridge --port 8765 --address 0.0.0.0
```
- 客户端：打开 Foxglove Studio，连接 `ws://<NX_IP>:8765`，选择需要的话题进行可视化。
- 与自研前端切换：
  - 自研前端用于“控制面 + 运营需求”；
  - Foxglove 作为“通用可视化兜底”，在弱网/故障或临时调试时启用。

---

## 安全基线与部署建议

- 网络入口：优先私网/VPN（WireGuard/Tailscale）；公网暴露需反向代理(Nginx/Caddy) + TLS。
- 认证与授权：JWT短期令牌+刷新/轮转；可选mTLS；最小权限分离（只读与控制用户）。
- 防护：IP白名单、速率限制（如/recording与/control 60 rpm）、WAF/CSRF防护、审计日志（谁/何时/何命令/结果）。
- 机密管理：环境变量+只读挂载，避免秘钥进镜像；定期轮换。

## 降级与自愈策略

- 弱网/拥塞：优先降帧→降分辨率→暂停点云→仅保留系统状态。
- 资源紧张：监控CPU/GPU/内存阈值，自动降低视频/点云开销；必要时暂停非关键流。
- 重连与限流：WS断线指数退避重连；并发连接>阈值时拒绝新连接或降级。
- 超时回滚：控制命令超时自动撤销，保持系统一致性。

## 进程托管与日志

- 使用systemd托管各服务，自动拉起与失败重启；日志收敛至journald或集中式日志。

示例（`/etc/systemd/system/ros-monitor-backend.service`）

```
[Unit]
Description=ROS Monitor Backend
After=network-online.target
Wants=network-online.target

[Service]
WorkingDirectory=/home/ycs/work/ikinghandbot/ros_monitor_backend
Environment=ROS_MASTER_URI=http://localhost:11311
ExecStart=/usr/bin/env bash -lc 'source /opt/ros/noetic/setup.bash && . .venv/bin/activate && uvicorn src.main:app --host 0.0.0.0 --port 8000'
Restart=on-failure
RestartSec=5
LimitNOFILE=65535

[Install]
WantedBy=multi-user.target
```

## 时钟同步与延迟监控

- 时钟：NX作为局域网NTP/chrony基准，所有设备指向；记录偏差并在UI显示。
- 时间戳：统一用ROS Time/稳态NTP时间；推送携带采集与发送时间，前端计算端到端延迟。

## 现成工具优先策略

- 快速监控：引入Foxglove Studio + foxglove-bridge 作为备选可视化；自研前端聚焦控制面与运营需求。

---

## 开发规范与最佳实践

### 代码质量标准

#### Python代码规范
- **PEP 8**: 严格遵循PEP 8代码风格
- **类型注解**: 使用typing模块进行类型标注
- **文档字符串**: 使用Google风格的docstring
- **异常处理**: 具体的异常类型和错误消息
- **日志记录**: 使用结构化日志记录

```python
# 示例代码规范
from typing import Dict, List, Optional
import logging

logger = logging.getLogger(__name__)

class SensorManager:
    """传感器管理器
    
    负责管理所有传感器的连接、数据获取和状态监控。
    
    Attributes:
        sensors: 传感器字典，key为传感器ID
        active: 管理器是否处于活动状态
    """
    
    def __init__(self, config: Dict[str, Any]) -> None:
        self.sensors: Dict[str, Sensor] = {}
        self.active: bool = False
        self._config = config
        
    async def initialize_sensors(self) -> None:
        """初始化所有传感器
        
        Raises:
            SensorInitializationError: 传感器初始化失败
        """
        try:
            for sensor_id, sensor_config in self._config.items():
                sensor = await self._create_sensor(sensor_id, sensor_config)
                self.sensors[sensor_id] = sensor
                logger.info(f"Sensor {sensor_id} initialized successfully")
        except Exception as e:
            logger.error(f"Failed to initialize sensors: {e}")
            raise SensorInitializationError(f"Initialization failed: {e}")
```

#### TypeScript代码规范
- **严格模式**: 开启TypeScript strict模式
- **接口定义**: 明确的接口和类型定义
- **函数式编程**: 优先使用函数式编程风格
- **组件规范**: 使用函数组件和Hooks

```typescript
// 示例接口定义
interface SensorData {
  readonly id: string;
  readonly type: SensorType;
  readonly timestamp: number;
  readonly data: unknown;
  readonly status: SensorStatus;
}

interface SensorStore {
  sensors: Record<string, SensorData>;
  updateSensorData: (id: string, data: Partial<SensorData>) => void;
  getSensorById: (id: string) => SensorData | undefined;
}

// 示例React组件
interface SensorViewerProps {
  sensorId: string;
  className?: string;
  onStatusChange?: (status: SensorStatus) => void;
}

export const SensorViewer: React.FC<SensorViewerProps> = ({ 
  sensorId, 
  className,
  onStatusChange 
}) => {
  const sensor = useSensorStore(state => state.getSensorById(sensorId));
  
  useEffect(() => {
    if (sensor && onStatusChange) {
      onStatusChange(sensor.status);
    }
  }, [sensor?.status, onStatusChange]);
  
  if (!sensor) {
    return <div className={className}>Sensor not found</div>;
  }
  
  return (
    <div className={className}>
      {/* 组件内容 */}
    </div>
  );
};
```

### 测试策略

#### 后端测试

```python
# tests/test_api/test_system.py
import pytest
from fastapi.testclient import TestClient
from src.main import app

client = TestClient(app)

class TestSystemAPI:
    def test_system_status(self):
        """测试系统状态API"""
        response = client.get("/api/v1/system/status")
        assert response.status_code == 200
        
        data = response.json()
        assert "success" in data
        assert "data" in data
        assert "ros_master" in data["data"]
        
    def test_system_control_unauthorized(self):
        """测试未授权的系统控制请求"""
        response = client.post("/api/v1/system/control", json={
            "action": "start_all"
        })
        assert response.status_code == 401

    @pytest.mark.asyncio
    async def test_websocket_connection(self):
        """测试WebSocket连接"""
        with client.websocket_connect("/ws/test_client") as websocket:
            # 发送认证消息
            websocket.send_json({
                "type": "auth",
                "token": "test_token"
            })
            
            # 接收确认消息
            data = websocket.receive_json()
            assert data["type"] == "connected"
```

#### 前端测试

```typescript
// src/components/__tests__/CameraViewer.test.tsx
import { render, screen, fireEvent } from '@testing-library/react';
import { CameraViewer } from '../camera/CameraViewer';
import { useSensorStore } from '../../stores/useSensorStore';

// Mock Zustand store
jest.mock('../../stores/useSensorStore');

describe('CameraViewer', () => {
  beforeEach(() => {
    (useSensorStore as jest.Mock).mockReturnValue({
      cameras: {
        left_camera: {
          imageData: 'base64_image_data',
          timestamp: Date.now() / 1000,
          width: 640,
          height: 480,
          encoding: 'jpeg'
        }
      }
    });
  });

  test('renders camera viewer with title', () => {
    render(<CameraViewer cameraId="left_camera" />);
    expect(screen.getByText('左相机')).toBeInTheDocument();
  });

  test('toggles streaming when switch is clicked', () => {
    render(<CameraViewer cameraId="left_camera" />);
    
    const streamSwitch = screen.getByRole('switch');
    fireEvent.click(streamSwitch);
    
    // 验证订阅行为
    expect(mockWebSocketService.subscribe).toHaveBeenCalledWith(['left_camera']);
  });
});
```

---

## 性能优化建议

### 后端性能优化

1. **数据压缩**: 使用gzip/lz4压缩点云数据
2. **连接池**: 使用连接池管理WebSocket连接
3. **缓存机制**: Redis缓存频繁访问的数据
4. **异步处理**: 使用asyncio处理并发请求
5. **资源限制**: 限制并发连接数和数据传输频率

### 前端性能优化

1. **虚拟化**: 大量数据使用虚拟滚动
2. **懒加载**: 按需加载组件和资源
3. **缓存策略**: 合理使用浏览器缓存
4. **Web Workers**: 计算密集任务使用Worker
5. **代码分割**: 路由级别的代码分割

---

## 总结

本开发指南提供了基于**HTTP API + WebSocket混合方案**的完整实现方案，包含：

✅ **完整的系统架构设计**
✅ **详细的API接口规范** 
✅ **核心代码实现示例**
✅ **容器化部署方案**
✅ **开发规范和最佳实践**

**预期开发时间**: 4周
**推荐团队配置**: 1名全栈开发工程师 + 1名测试工程师

按照此指南进行开发，可以构建出功能完整、性能优异、易于维护的ROS远程监控系统。

---

**文档版本**: v1.0  
**创建日期**: 2024年12月  
**适用版本**: ROS Noetic, Ubuntu 20.04  
**维护者**: IKing Handbot项目组
