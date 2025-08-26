#!/usr/bin/env python3
"""
ROS监控系统集成测试脚本
测试后端服务、WebSocket连接和相机数据流
"""

import asyncio
import aiohttp
import json
import time
import logging
from typing import Dict, Any

# 配置日志
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class SystemTester:
    def __init__(self, base_url: str = "http://localhost:8000"):
        self.base_url = base_url
        self.session = None
        
    async def __aenter__(self):
        self.session = aiohttp.ClientSession()
        return self
        
    async def __aexit__(self, exc_type, exc_val, exc_tb):
        if self.session:
            await self.session.close()
    
    async def test_health_check(self):
        """测试健康检查API"""
        logger.info("=== 测试健康检查API ===")
        
        try:
            async with self.session.get(f"{self.base_url}/api/v1/health") as response:
                if response.status == 200:
                    data = await response.json()
                    logger.info(f"✅ 健康检查通过: {data}")
                    return True
                else:
                    logger.error(f"❌ 健康检查失败: HTTP {response.status}")
                    return False
        except Exception as e:
            logger.error(f"❌ 健康检查异常: {e}")
            return False
    
    async def test_system_status(self):
        """测试系统状态API"""
        logger.info("=== 测试系统状态API ===")
        
        try:
            async with self.session.get(f"{self.base_url}/api/v1/system/status") as response:
                if response.status == 200:
                    data = await response.json()
                    logger.info(f"✅ 系统状态获取成功: {data}")
                    return True
                else:
                    logger.error(f"❌ 系统状态获取失败: HTTP {response.status}")
                    return False
        except Exception as e:
            logger.error(f"❌ 系统状态获取异常: {e}")
            return False
    
    async def test_websocket_connection(self):
        """测试WebSocket连接"""
        logger.info("=== 测试WebSocket连接 ===")
        
        try:
            import websockets
            
            client_id = f"test_client_{int(time.time())}"
            ws_url = f"ws://localhost:8000/ws/{client_id}"
            
            async with websockets.connect(ws_url) as websocket:
                logger.info(f"✅ WebSocket连接成功: {ws_url}")
                
                # 等待连接确认
                try:
                    message = await asyncio.wait_for(websocket.recv(), timeout=5.0)
                    data = json.loads(message)
                    if data.get("type") == "connected":
                        logger.info("✅ 收到连接确认消息")
                        
                        # 测试订阅
                        subscribe_msg = {
                            "type": "subscribe",
                            "topics": ["camera"]
                        }
                        await websocket.send(json.dumps(subscribe_msg))
                        logger.info("✅ 发送订阅消息")
                        
                        # 等待订阅确认
                        try:
                            sub_message = await asyncio.wait_for(websocket.recv(), timeout=5.0)
                            sub_data = json.loads(sub_message)
                            if sub_data.get("type") == "subscribed":
                                logger.info("✅ 收到订阅确认消息")
                            else:
                                logger.warning(f"⚠️ 收到非订阅确认消息: {sub_data}")
                        except asyncio.TimeoutError:
                            logger.warning("⚠️ 订阅确认超时")
                        
                        # 等待相机数据
                        logger.info("等待相机数据...")
                        try:
                            camera_message = await asyncio.wait_for(websocket.recv(), timeout=10.0)
                            camera_data = json.loads(camera_message)
                            if camera_data.get("type") == "camera":
                                logger.info(f"✅ 收到相机数据: {camera_data.get('camera_id')}")
                            else:
                                logger.info(f"📨 收到其他消息: {camera_data.get('type')}")
                        except asyncio.TimeoutError:
                            logger.warning("⚠️ 相机数据接收超时")
                        
                        return True
                    else:
                        logger.warning(f"⚠️ 收到非连接确认消息: {data}")
                        return False
                        
                except asyncio.TimeoutError:
                    logger.error("❌ 连接确认超时")
                    return False
                    
        except ImportError:
            logger.error("❌ websockets库未安装，无法测试WebSocket")
            return False
        except Exception as e:
            logger.error(f"❌ WebSocket测试异常: {e}")
            return False
    
    async def run_all_tests(self):
        """运行所有测试"""
        logger.info("🚀 开始ROS监控系统集成测试")
        
        results = []
        
        # 健康检查测试
        health_result = await self.test_health_check()
        results.append(("健康检查", health_result))
        
        # 系统状态测试
        status_result = await self.test_system_status()
        results.append(("系统状态", status_result))
        
        # WebSocket连接测试
        ws_result = await self.test_websocket_connection()
        results.append(("WebSocket连接", ws_result))
        
        # 输出测试结果
        logger.info("\n=== 测试结果汇总 ===")
        success_count = 0
        for test_name, result in results:
            status = "✅ 通过" if result else "❌ 失败"
            logger.info(f"{test_name}: {status}")
            if result:
                success_count += 1
        
        total_tests = len(results)
        logger.info(f"\n总计: {success_count}/{total_tests} 项测试通过")
        
        if success_count == total_tests:
            logger.info("🎉 所有测试通过！系统运行正常")
        else:
            logger.warning("⚠️ 部分测试失败，请检查系统状态")
        
        return success_count == total_tests

async def main():
    """主函数"""
    logger.info("ROS监控系统集成测试工具")
    logger.info("请确保后端服务已启动 (python -m src.main)")
    
    async with SystemTester() as tester:
        try:
            success = await tester.run_all_tests()
            return success
        except KeyboardInterrupt:
            logger.info("测试被用户中断")
            return False
        except Exception as e:
            logger.error(f"测试执行错误: {e}")
            return False

if __name__ == "__main__":
    success = asyncio.run(main())
    exit(0 if success else 1)