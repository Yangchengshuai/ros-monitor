from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
import uvicorn
import asyncio
from contextlib import asynccontextmanager
import logging
import time

from src.websocket.connection_manager import ConnectionManager
from src.ros_bridge.node_manager import ROSNodeManager
from src.api.v1.data_collection import router as data_collection_router

# 配置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# 全局变量
ros_manager = None
connection_manager = ConnectionManager()

@asynccontextmanager
async def lifespan(app: FastAPI):
    """应用生命周期管理"""
    global ros_manager
    
    # 启动时初始化ROS
    logger.info("Starting ROS Monitor Backend...")
    try:
        ros_manager = ROSNodeManager()
        await ros_manager.initialize()
        logger.info("ROS Node Manager initialized successfully")
    except Exception as e:
        logger.error(f"Failed to initialize ROS Node Manager: {e}")
        ros_manager = None
    
    # 启动后台任务
    asyncio.create_task(background_broadcast())
    
    yield
    
    # 关闭时清理
    logger.info("Shutting down ROS Monitor Backend...")
    if ros_manager:
        await ros_manager.shutdown()

# 创建FastAPI应用
app = FastAPI(
    title="ROS Monitor Backend",
    description="ROS远程监控后端服务",
    version="1.0.0",
    lifespan=lifespan
)

# 中间件配置
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # 开发环境允许所有来源
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# 健康检查API
@app.get("/api/v1/health")
async def health_check():
    """健康检查接口"""
    ros_ready = ros_manager is not None and ros_manager.is_connected()
    return {
        "success": True,
        "message": "ok",
        "ros_ready": ros_ready,
        "timestamp": time.time(),
        "websocket_clients": connection_manager.get_client_count()
    }

# 系统状态API
@app.get("/api/v1/system/status")
async def system_status():
    """系统状态接口"""
    if ros_manager is None:
        return {
            "success": False,
            "message": "ROS Manager not initialized",
            "data": None
        }
    
    try:
        connection_info = ros_manager.get_connection_info()
        return {
            "success": True,
            "message": "System status retrieved successfully",
            "data": {
                "ros_connection": connection_info,
                "websocket_status": {
                    "total_clients": connection_manager.get_client_count(),
                    "subscription_info": connection_manager.get_subscription_info()
                },
                "timestamp": time.time()
            }
        }
    except Exception as e:
        logger.error(f"Error getting system status: {e}")
        return {
            "success": False,
            "message": f"Error retrieving system status: {str(e)}",
            "data": None
        }

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
    except Exception as e:
        logger.error(f"WebSocket error for {client_id}: {e}")
        connection_manager.disconnect(client_id)

async def handle_websocket_message(client_id: str, message: dict):
    """处理WebSocket消息"""
    try:
        msg_type = message.get("type")
        
        if msg_type == "subscribe":
            # 修复：从data字段中获取topics
            topics = message.get("data", {}).get("topics", [])
            await connection_manager.subscribe_topics(client_id, topics)
            logger.info(f"Client {client_id} subscribed to topics: {topics}")
            
            # 发送订阅确认 - 修复格式，添加data字段
            await connection_manager.send_personal_message({
                "type": "subscription_confirmed",
                "data": {
                    "topics": topics,
                    "message": f"已成功订阅话题: {', '.join(topics)}"
                },
                "timestamp": time.time()
            }, client_id)
            
        elif msg_type == "unsubscribe":
            # 修复：从data字段中获取topics
            topics = message.get("data", {}).get("topics", [])
            await connection_manager.unsubscribe_topics(client_id, topics)
            logger.info(f"Client {client_id} unsubscribed from topics: {topics}")
            
            # 发送取消订阅确认
            await connection_manager.send_personal_message({
                "type": "unsubscribed",
                "data": {
                    "topics": topics,
                    "message": f"已取消订阅话题: {', '.join(topics)}"
                },
                "timestamp": time.time()
            }, client_id)
            
        elif msg_type == "request_system_status":
            # 发送系统状态
            if ros_manager:
                connection_info = ros_manager.get_connection_info()
                system_status = {
                    "type": "system_status",
                    "data": {
                        "ros_ready": ros_manager.is_connected(),
                        "websocket_status": "connected",
                        "api_status": True,
                        "ros_info": connection_info
                    },
                    "timestamp": time.time()
                }
            else:
                system_status = {
                    "type": "system_status",
                    "data": {
                        "ros_ready": False,
                        "websocket_status": "connected",
                        "api_status": True,
                        "ros_info": None
                    },
                    "timestamp": time.time()
                }
            
            await connection_manager.send_personal_message(system_status, client_id)
            
        elif msg_type == "ping":
            # 响应ping消息
            await connection_manager.send_personal_message({
                "type": "pong",
                "data": {
                    "message": "pong"
                },
                "timestamp": time.time()
            }, client_id)
            
        elif msg_type == "camera_settings":
            # 处理相机设置更新
            if ros_manager:
                camera_id = message.get("camera_id")
                preview_height = message.get("preview_height")
                jpeg_quality = message.get("jpeg_quality")
                
                if camera_id and (preview_height is not None or jpeg_quality is not None):
                    success = ros_manager.update_camera_settings(camera_id, preview_height, jpeg_quality)
                    await connection_manager.send_personal_message({
                        "type": "camera_settings_updated",
                        "data": {
                            "camera_id": camera_id,
                            "success": success,
                            "message": "相机设置已更新" if success else "相机设置更新失败"
                        },
                        "timestamp": time.time()
                    }, client_id)
            else:
                await connection_manager.send_personal_message({
                    "type": "error",
                    "data": {
                        "message": "ROS管理器未初始化"
                    },
                    "timestamp": time.time()
                }, client_id)
                
        else:
            logger.warning(f"Unknown message type from {client_id}: {msg_type}")
            await connection_manager.send_personal_message({
                "type": "error",
                "data": {
                    "message": f"未知的消息类型: {msg_type}"
                },
                "timestamp": time.time()
            }, client_id)
            
    except Exception as e:
        logger.error(f"Error handling WebSocket message from {client_id}: {e}")
        await connection_manager.send_personal_message({
            "type": "error",
            "data": {
                "message": f"消息处理错误: {str(e)}"
            },
            "timestamp": time.time()
        }, client_id)

async def background_broadcast():
    """后台数据广播任务"""
    logger.info("🚀 后台数据广播任务已启动")
    
    while True:
        try:
            if ros_manager and ros_manager.is_connected():
                logger.debug("ROS管理器已连接，开始获取传感器数据...")
                
                # 获取最新的传感器数据
                camera_data = await ros_manager.get_latest_camera_data()
                lidar_data = await ros_manager.get_latest_lidar_data()
                imu_data = await ros_manager.get_latest_imu_data()
                
                # 推送给订阅的客户端
                if camera_data:
                    logger.info(f"📷 广播相机数据: {list(camera_data.keys())}")
                    for camera_id, data in camera_data.items():
                        message = {
                            "type": "camera",
                            "data": {
                                "camera_id": camera_id,
                                **data  # 展开相机数据的所有字段
                            },
                            "timestamp": time.time()
                        }
                        logger.info(f"📤 发送相机 {camera_id} 数据，帧数: {data.get('sequence', 0)}")
                        await connection_manager.broadcast_to_subscribers("camera", message)
                        logger.info(f"✅ 相机 {camera_id} 数据已广播，帧数: {data.get('sequence', 0)}")
                else:
                    logger.warning("⚠️ 没有相机数据可广播 - 检查ROS话题是否有数据")
                
                if lidar_data:
                    message = {
                        "type": "lidar",
                        "data": lidar_data,
                        "timestamp": time.time()
                    }
                    await connection_manager.broadcast_to_subscribers("lidar", message)
                    logger.debug("📡 激光雷达数据已广播")
                    
                if imu_data:
                    message = {
                        "type": "imu",
                        "data": imu_data,
                        "timestamp": time.time()
                    }
                    await connection_manager.broadcast_to_subscribers("imu", message)
                    logger.debug("📊 IMU数据已广播")
                    
            else:
                logger.warning("⚠️ ROS管理器未连接或未初始化")
                
        except Exception as e:
            logger.error(f"❌ 后台广播错误: {e}")
            import traceback
            logger.error(f"错误详情: {traceback.format_exc()}")
        
        await asyncio.sleep(0.1)  # 10Hz推送频率

# 注册数据采集路由
from src.api.v1.data_collection import router as data_collection_router
app.include_router(data_collection_router, prefix="/api/v1")

if __name__ == "__main__":
    import os
    
    # 从环境变量获取配置
    host = os.getenv("ROS_MONITOR_HOST", "0.0.0.0")
    port = int(os.getenv("ROS_MONITOR_BACKEND_PORT", "8000"))
    
    print(f"🚀 启动ROS监控后端服务...")
    print(f"   主机: {host}")
    print(f"   端口: {port}")
    
    uvicorn.run(
        "src.main:app",
        host=host,
        port=port,
        reload=False,
        log_level="info"
    )