# Epic: ROS数据采集远程控制功能

## 1. Epic概述

**Epic名称**: ROS数据采集远程控制功能  
**目标**: 在现有ROS远程监控系统基础上，实现通过Web界面远程控制数据采集的开始和停止  
**业务价值**: 实现无人值守的数据采集作业，支持远程操作和监控  
**技术挑战**: 安全的脚本执行、实时状态监控、多客户端同步

## 2. 业务需求分解

### 2.1 核心需求 (Core Requirements)

#### US-001: 远程启动数据采集
**作为** 系统操作员  
**我想要** 通过Web界面点击按钮启动数据采集  
**以便于** 远程启动数据采集作业而无需物理访问设备

**验收标准**:
- [ ] 点击"开始采集"按钮后，系统执行 `script/start_all.sh`
- [ ] 执行成功后在界面显示"采集中"状态
- [ ] 执行失败显示错误信息和重试选项
- [ ] 支持局域网内任意设备访问和控制

#### US-002: 远程停止数据采集
**作为** 系统操作员  
**我想要** 通过Web界面点击按钮停止数据采集  
**以便于** 远程结束数据采集作业

**验收标准**:
- [ ] 点击"停止采集"按钮后，系统执行 `script/stop_all.sh`
- [ ] 执行成功后在界面显示"已停止"状态
- [ ] 支持强制停止功能（处理异常进程）

#### US-003: 实时状态监控
**作为** 系统操作员  
**我想要** 实时查看数据采集状态  
**以便于** 了解当前采集作业的进度和健康状况

**验收标准**:
- [ ] 显示当前采集状态（运行中/已停止/错误）
- [ ] 显示采集开始时间和运行时长
- [ ] 显示最近的状态更新时间
- [ ] 支持自动刷新状态（每5秒）

### 2.2 用户体验需求 (UX Requirements)

#### US-004: 直观的控制界面
**作为** 系统操作员  
**我想要** 直观易用的控制界面  
**以便于** 无需培训即可操作

**验收标准**:
- [ ] 按钮状态清晰（启用/禁用/加载中）
- [ ] 使用颜色编码状态（绿色运行中，红色停止，灰色不可用）
- [ ] 提供操作反馈（成功/失败提示）
- [ ] 支持键盘快捷键操作

#### US-005: 多设备访问支持
**作为** 系统操作员  
**我想要** 从任何设备访问控制界面  
**以便于** 使用手机、平板、电脑都能操作

**验收标准**:
- [ ] 响应式设计，适配不同屏幕尺寸
- [ ] 支持触摸操作
- [ ] 移动端友好的按钮大小

### 2.3 技术需求 (Technical Requirements)

#### US-006: 安全脚本执行
**作为** 系统管理员  
**我想要** 确保脚本执行的安全性  
**以便于** 防止恶意操作和系统破坏

**验收标准**:
- [ ] 验证脚本路径合法性（限制在指定目录）
- [ ] 检查脚本文件权限
- [ ] 记录所有执行操作的审计日志
- [ ] 限制并发执行（防止重复启动）

#### US-007: 错误处理和恢复
**作为** 系统操作员  
**我想要** 系统能够优雅处理错误  
**以便于** 在异常情况下能够恢复操作

**验收标准**:
- [ ] 进程异常退出时自动检测并提示
- [ ] 提供错误详情和解决建议
- [ ] 支持手动清理僵尸进程
- [ ] 网络断开后自动重连

## 3. 技术实现架构

### 3.1 系统架构图

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   Web Frontend  │    │   FastAPI Backend │    │   OS Scripts    │
│                 │    │                  │    │                 │
│ ┌─────────────┐ │    │ ┌──────────────┐ │    │ ┌─────────────┐ │
│ │  React UI   │ │◄──►│ │  API Routes  │ │◄──►│ │ start_all.sh│ │
│ │  Components │ │    │ │  WebSocket   │ │    │ │ stop_all.sh │ │
│ └─────────────┘ │    │ │  ScriptExec  │ │    │ └─────────────┘ │
└─────────────────┘    │ └──────────────┘ │    └─────────────────┘
                       └──────────────────┘
                                │
                       ┌──────────────────┐
                       │   Process        │
                       │   Monitoring     │
                       └──────────────────┘
```

### 3.2 核心组件设计

#### 3.2.1 后端组件

**ScriptExecutor服务**:
```python
class ScriptExecutor:
    """脚本执行服务"""
    
    def __init__(self):
        self.active_processes: Dict[str, asyncio.subprocess.Process] = {}
        self.execution_log: List[ExecutionRecord] = []
    
    async def execute_script(self, script_path: str, timeout: int = 30) -> ExecutionResult:
        """安全执行脚本"""
        
    async def get_process_status(self, script_name: str) -> ProcessStatus:
        """获取进程状态"""
```

**DataCollectionManager服务**:
```python
class DataCollectionManager:
    """数据采集管理器"""
    
    def __init__(self, script_executor: ScriptExecutor):
        self.executor = script_executor
        self.status_cache: Dict[str, Any] = {}
        self.monitoring_task: Optional[asyncio.Task] = None
    
    async def start_collection(self) -> bool:
        """启动数据采集"""
        
    async def stop_collection(self) -> bool:
        """停止数据采集"""
        
    async def get_status(self) -> DataCollectionStatus:
        """获取采集状态"""
```

#### 3.2.2 前端组件

**DataCollectionControl组件**:
```typescript
interface DataCollectionControlProps {
  onStatusChange?: (status: DataCollectionStatus) => void;
  className?: string;
}

const DataCollectionControl: React.FC<DataCollectionControlProps> = ({
  onStatusChange,
  className
}) => {
  const [status, setStatus] = useState<DataCollectionStatus>(initialStatus);
  const [loading, setLoading] = useState(false);
  
  const handleStartCollection = async () => {
    // 启动采集逻辑
  };
  
  const handleStopCollection = async () => {
    // 停止采集逻辑
  };
};
```

### 3.3 API端点设计

#### 3.3.1 REST API

**启动数据采集**:
```http
POST /api/v1/data-collection/start
Content-Type: application/json

{
  "script": "start_all.sh",
  "parameters": {},
  "timeout": 30
}

Response:
{
  "success": true,
  "message": "数据采集已启动",
  "data": {
    "process_id": 1234,
    "start_time": "2024-01-01T12:00:00Z",
    "script_path": "script/start_all.sh"
  }
}
```

**停止数据采集**:
```http
POST /api/v1/data-collection/stop
Content-Type: application/json

{
  "force": false
}

Response:
{
  "success": true,
  "message": "数据采集已停止",
  "data": {
    "stop_time": "2024-01-01T12:30:00Z",
    "duration": 1800
  }
}
```

**获取采集状态**:
```http
GET /api/v1/data-collection/status

Response:
{
  "success": true,
  "data": {
    "is_running": true,
    "process_id": 1234,
    "start_time": "2024-01-01T12:00:00Z",
    "duration": 1800,
    "script_path": "script/start_all.sh",
    "last_update": "2024-01-01T12:30:15Z"
  }
}
```

#### 3.3.2 WebSocket消息

**状态广播**:
```typescript
interface DataCollectionStatusMessage {
  type: 'data_collection_status';
  data: {
    is_running: boolean;
    process_id: number | null;
    start_time: string | null;
    duration: number;
    script_path: string;
    last_update: string;
  };
}
```

### 3.4 状态管理架构

#### 3.4.1 后端状态管理

**状态机设计**:
```
[Stopped] → [Starting] → [Running] → [Stopping] → [Stopped]
   ↑            ↓           ↓           ↓           ↑
   └────────────┴───────────┴───────────┴───────────┘
                    [Error]
```

**状态存储**:
- 内存缓存：当前状态、进程信息
- 定期持久化：执行日志、错误记录
- 实时监控：进程存活状态

#### 3.4.2 前端状态管理

**Zustand Store设计**:
```typescript
interface DataCollectionStore {
  status: DataCollectionStatus;
  isLoading: boolean;
  error: string | null;
  
  // Actions
  startCollection: () => Promise<void>;
  stopCollection: () => Promise<void>;
  refreshStatus: () => Promise<void>;
  subscribeToUpdates: () => void;
}
```

### 3.5 错误处理架构

#### 3.5.1 错误分类

**系统级错误**:
- 脚本文件不存在
- 权限不足
- 系统资源不足

**应用级错误**:
- 进程启动失败
- 进程意外退出
- 网络连接中断

**用户级错误**:
- 重复启动
- 无效操作（停止未运行的进程）

#### 3.5.2 错误恢复机制

**自动恢复**:
- 网络重连后自动同步状态
- 进程监控自动检测异常

**手动恢复**:
- 提供强制停止功能
- 提供状态重置功能
- 提供错误详情查看

## 4. 技术实现任务分解

### 4.1 Sprint 1: 后端基础功能

#### Task-BE-001: ScriptExecutor服务实现
**描述**: 实现安全的脚本执行服务
**工作量**: 4小时
**技术要点**:
- 使用 `asyncio.create_subprocess_exec` 实现异步执行
- 添加超时机制（默认30秒）
- 实现进程监控和清理

#### Task-BE-002: DataCollectionManager实现
**描述**: 实现数据采集管理器
**工作量**: 3小时
**技术要点**:
- 集成ScriptExecutor服务
- 实现状态管理和缓存
- 添加状态广播机制

#### Task-BE-003: REST API端点实现
**描述**: 实现数据采集控制API
**工作量**: 2小时
**技术要点**:
- FastAPI路由设计
- 请求验证和错误处理
- 响应格式标准化

#### Task-BE-004: WebSocket集成
**描述**: 添加WebSocket状态广播
**工作量**: 2小时
**技术要点**:
- WebSocket消息类型扩展
- 实时状态推送
- 客户端状态同步

### 4.2 Sprint 2: 前端界面实现

#### Task-FE-001: API服务层扩展
**描述**: 扩展API服务支持数据采集控制
**工作量**: 1小时
**技术要点**:
- 添加新的API调用方法
- 错误处理和重试机制
- 类型定义完善

#### Task-FE-002: 数据采集控制组件
**描述**: 创建数据采集控制UI组件
**工作量**: 3小时
**技术要点**:
- React组件设计
- Ant Design集成
- 响应式布局

#### Task-FE-003: 状态管理集成
**描述**: 集成Zustand状态管理
**工作量**: 2小时
**技术要点**:
- 状态store设计
- 异步action实现
- WebSocket状态同步

#### Task-FE-004: 界面集成和优化
**描述**: 集成到现有界面并优化用户体验
**工作量**: 2小时
**技术要点**:
- 与CameraControlPanel集成
- 加载状态处理
- 错误提示优化

### 4.3 Sprint 3: 测试和部署

#### Task-TEST-001: 单元测试
**描述**: 编写核心功能单元测试
**工作量**: 3小时
**测试范围**:
- ScriptExecutor服务测试
- API端点测试
- 前端组件测试

#### Task-TEST-002: 集成测试
**描述**: 端到端功能测试
**工作量**: 2小时
**测试场景**:
- 完整采集流程测试
- 异常场景测试
- 多客户端并发测试

#### Task-DEPLOY-001: 部署配置
**描述**: 配置生产环境部署
**工作量**: 1小时
**配置内容**:
- 网络访问配置
- 权限设置
- 日志配置

## 5. 技术风险和缓解策略

### 5.1 安全风险
**风险**: 脚本执行权限过大
**缓解**:
- 限制脚本执行路径
- 使用专用用户运行
- 添加操作审计日志

### 5.2 稳定性风险
**风险**: 进程管理异常
**缓解**:
- 进程监控和自动清理
- 超时机制
- 状态定期同步

### 5.3 网络风险
**风险**: 网络中断导致状态不一致
**缓解**:
- WebSocket重连机制
- 状态定期刷新
- 客户端状态验证

## 6. 性能指标

### 6.1 响应时间
- API响应时间: < 2秒
- WebSocket消息延迟: < 1秒
- 界面更新延迟: < 500ms

### 6.2 并发能力
- 支持并发用户数: 10个
- 状态广播频率: 每5秒
- 状态查询频率: 每秒1次

### 6.3 资源使用
- 内存使用增量: < 50MB
- CPU使用率增量: < 5%
- 网络带宽: < 1KB/s（状态更新）

## 7. 未来扩展规划

### 7.1 功能扩展
- **计划任务**: 支持定时启动/停止采集
- **数据存储**: 集成数据上传功能
- **监控告警**: 异常情况自动通知

### 7.2 架构扩展
- **分布式部署**: 支持多节点采集
- **微服务化**: 数据采集服务独立部署
- **API网关**: 统一入口和认证

### 7.3 用户体验扩展
- **历史记录**: 查看操作历史
- **统计分析**: 采集数据统计
- **个性化配置**: 用户偏好设置

## 8. 验收标准

### 8.1 功能验收
- [ ] 能够通过Web界面启动数据采集
- [ ] 能够通过Web界面停止数据采集
- [ ] 能够实时查看采集状态
- [ ] 支持局域网内多设备访问
- [ ] 提供完整的错误处理和用户反馈

### 8.2 技术验收
- [ ] 所有API端点正常工作
- [ ] WebSocket消息正确广播
- [ ] 单元测试覆盖率 > 80%
- [ ] 性能指标达标
- [ ] 安全审计通过

### 8.3 文档验收
- [ ] 用户操作手册完整
- [ ] API文档完整
- [ ] 部署指南详细
- [ ] 故障排查文档完整

---

**Epic Owner**: 系统架构师  
**技术负责人**: 后端/前端开发工程师  
**预计工期**: 2-3周  
**优先级**: 高  
**状态**: 待启动