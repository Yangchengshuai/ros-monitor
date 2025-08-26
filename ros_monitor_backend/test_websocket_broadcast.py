#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
测试WebSocket消息广播功能
"""

import asyncio
import json
import logging
from websockets import connect

# 配置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

async def test_websocket_broadcast():
    """测试WebSocket消息广播"""
    try:
        logger.info("开始测试WebSocket消息广播...")
        
        # 连接到WebSocket
        uri = "ws://localhost:8000/ws/test_client_123"
        async with connect(uri) as websocket:
            logger.info("WebSocket连接成功")
            
            # 订阅相机话题
            subscribe_message = {
                "type": "subscribe",
                "topics": ["camera"]
            }
            await websocket.send(json.dumps(subscribe_message))
            logger.info("已发送订阅消息")
            
            # 等待订阅确认
            response = await websocket.recv()
            logger.info(f"收到响应: {response}")
            
            # 等待相机数据
            logger.info("等待相机数据...")
            timeout = 30  # 30秒超时
            
            start_time = asyncio.get_event_loop().time()
            while True:
                try:
                    # 设置接收超时
                    data = await asyncio.wait_for(websocket.recv(), timeout=5.0)
                    logger.info(f"收到数据: {data[:200]}...")  # 只显示前200个字符
                    
                    # 解析JSON
                    try:
                        message = json.loads(data)
                        if message.get("type") == "camera":
                            logger.info(f"✅ 收到相机数据: camera_id={message.get('camera_id')}")
                            logger.info(f"   帧数: {message.get('data', {}).get('sequence', 'N/A')}")
                            logger.info(f"   时间戳: {message.get('timestamp', 'N/A')}")
                            break
                    except json.JSONDecodeError:
                        logger.warning(f"无法解析JSON: {data}")
                        
                except asyncio.TimeoutError:
                    current_time = asyncio.get_event_loop().time()
                    elapsed = current_time - start_time
                    logger.info(f"等待中... 已等待 {elapsed:.1f} 秒")
                    
                    if elapsed > timeout:
                        logger.error("❌ 超时：没有收到相机数据")
                        break
                        
    except Exception as e:
        logger.error(f"测试失败: {e}")
        import traceback
        logger.error(f"错误详情: {traceback.format_exc()}")

if __name__ == "__main__":
    asyncio.run(test_websocket_broadcast())
