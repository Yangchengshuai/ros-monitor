#!/usr/bin/env python3
"""
WebSocket服务集成测试脚本
测试相机数据推送和WebSocket连接管理
"""

import asyncio
import websockets
import json
import time
import logging
from typing import Dict, Any

# 配置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class WebSocketTester:
    def __init__(self, uri: str = "ws://localhost:8000"):
        self.uri = uri
        self.client_id = f"test_client_{int(time.time())}"
        self.ws = None
        self.received_messages = []
        self.connection_status = False
        
    async def connect(self):
        """建立WebSocket连接"""
        try:
            self.ws = await websockets.connect(f"{self.uri}/ws/{self.client_id}")
            self.connection_status = True
            logger.info(f"WebSocket connected with client ID: {self.client_id}")
            return True
        except Exception as e:
            logger.error(f"Failed to connect: {e}")
            return False
    
    async def disconnect(self):
        """断开WebSocket连接"""
        if self.ws:
            await self.ws.close()
            self.connection_status = False
            logger.info("WebSocket disconnected")
    
    async def send_message(self, message: Dict[str, Any]):
        """发送消息"""
        if not self.connection_status:
            logger.error("WebSocket not connected")
            return False
        
        try:
            await self.ws.send(json.dumps(message))
            logger.info(f"Sent message: {message}")
            return True
        except Exception as e:
            logger.error(f"Failed to send message: {e}")
            return False
    
    async def receive_messages(self, timeout: int = 10):
        """接收消息"""
        if not self.connection_status:
            logger.error("WebSocket not connected")
            return
        
        try:
            # 设置接收超时
            await asyncio.wait_for(self._receive_loop(), timeout)
        except asyncio.TimeoutError:
            logger.info("Message receiving timeout")
    
    async def _receive_loop(self):
        """消息接收循环"""
        while self.connection_status:
            try:
                message = await self.ws.recv()
                parsed_message = json.loads(message)
                self.received_messages.append(parsed_message)
                logger.info(f"Received message: {parsed_message}")
                
                # 处理特定类型的消息
                await self._handle_message(parsed_message)
                
            except websockets.exceptions.ConnectionClosed:
                logger.info("WebSocket connection closed")
                break
            except Exception as e:
                logger.error(f"Error receiving message: {e}")
                break
    
    async def _handle_message(self, message: Dict[str, Any]):
        """处理接收到的消息"""
        msg_type = message.get("type")
        
        if msg_type == "connected":
            logger.info("✅ Connection confirmed")
        elif msg_type == "subscribed":
            logger.info(f"✅ Subscription confirmed for topics: {message.get('topics', [])}")
        elif msg_type == "camera":
            logger.info(f"📷 Camera data received for: {message.get('camera_id')}")
        elif msg_type == "system_status":
            logger.info("📊 System status received")
        elif msg_type == "error":
            logger.error(f"❌ Error message: {message.get('message')}")
        else:
            logger.info(f"📨 Unknown message type: {msg_type}")
    
    async def test_basic_connection(self):
        """测试基本连接"""
        logger.info("=== 测试基本连接 ===")
        
        # 连接测试
        if not await self.connect():
            return False
        
        # 等待连接确认
        await asyncio.sleep(1)
        
        # 检查是否收到连接确认
        connection_messages = [msg for msg in self.received_messages if msg.get("type") == "connected"]
        if connection_messages:
            logger.info("✅ 基本连接测试通过")
            return True
        else:
            logger.error("❌ 基本连接测试失败")
            return False
    
    async def test_subscription(self):
        """测试话题订阅"""
        logger.info("=== 测试话题订阅 ===")
        
        # 订阅相机话题
        subscribe_message = {
            "type": "subscribe",
            "topics": ["camera"]
        }
        
        if await self.send_message(subscribe_message):
            # 等待订阅确认
            await asyncio.sleep(2)
            
            # 检查是否收到订阅确认
            subscription_messages = [msg for msg in self.received_messages if msg.get("type") == "subscribed"]
            if subscription_messages:
                logger.info("✅ 话题订阅测试通过")
                return True
            else:
                logger.error("❌ 话题订阅测试失败")
                return False
        else:
            logger.error("❌ 发送订阅消息失败")
            return False
    
    async def test_system_status(self):
        """测试系统状态请求"""
        logger.info("=== 测试系统状态请求 ===")
        
        # 请求系统状态
        status_message = {
            "type": "request_system_status"
        }
        
        if await self.send_message(status_message):
            # 等待系统状态响应
            await asyncio.sleep(2)
            
            # 检查是否收到系统状态
            status_messages = [msg for msg in self.received_messages if msg.get("type") == "system_status"]
            if subscription_messages:
                logger.info("✅ 系统状态请求测试通过")
                return True
            else:
                logger.error("❌ 系统状态请求测试失败")
                return False
        else:
            logger.error("❌ 发送系统状态请求失败")
            return False
    
    async def test_camera_data_reception(self):
        """测试相机数据接收"""
        logger.info("=== 测试相机数据接收 ===")
        
        # 等待相机数据
        await asyncio.sleep(5)
        
        # 检查是否收到相机数据
        camera_messages = [msg for msg in self.received_messages if msg.get("type") == "camera"]
        if camera_messages:
            logger.info(f"✅ 相机数据接收测试通过，收到 {len(camera_messages)} 条数据")
            return True
        else:
            logger.warning("⚠️ 未收到相机数据，可能是后端未推送或话题未订阅")
            return False
    
    async def run_all_tests(self):
        """运行所有测试"""
        logger.info("🚀 开始WebSocket集成测试")
        
        try:
            # 基本连接测试
            if not await self.test_basic_connection():
                return False
            
            # 话题订阅测试
            if not await self.test_subscription():
                return False
            
            # 系统状态请求测试
            if not await self.test_system_status():
                return False
            
            # 相机数据接收测试
            await self.test_camera_data_reception()
            
            logger.info("🎉 所有测试完成")
            return True
            
        except Exception as e:
            logger.error(f"测试过程中发生错误: {e}")
            return False
        finally:
            await self.disconnect()
    
    def print_summary(self):
        """打印测试摘要"""
        logger.info("\n=== 测试摘要 ===")
        logger.info(f"总连接时间: {len(self.received_messages)} 秒")
        logger.info(f"接收消息数: {len(self.received_messages)}")
        
        # 按类型统计消息
        message_types = {}
        for msg in self.received_messages:
            msg_type = msg.get("type", "unknown")
            message_types[msg_type] = message_types.get(msg_type, 0) + 1
        
        logger.info("消息类型统计:")
        for msg_type, count in message_types.items():
            logger.info(f"  {msg_type}: {count} 条")
        
        # 显示最后几条消息
        if self.received_messages:
            logger.info("\n最后3条消息:")
            for msg in self.received_messages[-3:]:
                logger.info(f"  {msg}")

async def main():
    """主函数"""
    tester = WebSocketTester()
    
    try:
        success = await tester.run_all_tests()
        tester.print_summary()
        
        if success:
            logger.info("✅ 集成测试成功")
        else:
            logger.error("❌ 集成测试失败")
            
    except KeyboardInterrupt:
        logger.info("测试被用户中断")
    except Exception as e:
        logger.error(f"测试执行错误: {e}")
    finally:
        await tester.disconnect()

if __name__ == "__main__":
    asyncio.run(main())