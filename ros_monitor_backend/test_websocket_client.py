#!/usr/bin/env python3
"""
WebSocket客户端测试脚本
"""

import asyncio
import websockets
import json
import time

async def test_websocket():
    uri = "ws://localhost:8000/ws/test_client_python"
    
    print("🚀 开始WebSocket连接测试...")
    print(f"连接地址: {uri}")
    
    try:
        async with websockets.connect(uri) as websocket:
            print("✅ WebSocket连接成功!")
            
            # 测试1: 发送订阅消息
            print("\n📤 测试1: 发送相机订阅消息")
            subscribe_msg = {
                "type": "subscribe",
                "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ"),
                "data": {"topics": ["camera"]}
            }
            await websocket.send(json.dumps(subscribe_msg))
            print(f"发送: {json.dumps(subscribe_msg, indent=2)}")
            
            # 等待响应
            try:
                response = await asyncio.wait_for(websocket.recv(), timeout=5.0)
                print(f"📨 收到响应: {response}")
            except asyncio.TimeoutError:
                print("⏰ 5秒内未收到响应")
            
            # 测试2: 发送IMU订阅消息
            print("\n📤 测试2: 发送IMU订阅消息")
            imu_msg = {
                "type": "subscribe",
                "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ"),
                "data": {"topics": ["imu"]}
            }
            await websocket.send(json.dumps(imu_msg))
            print(f"发送: {json.dumps(imu_msg, indent=2)}")
            
            # 测试3: 发送自定义消息
            print("\n📤 测试3: 发送自定义消息")
            custom_msg = {
                "type": "ping",
                "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ"),
                "data": {"message": "hello from python client"}
            }
            await websocket.send(json.dumps(custom_msg))
            print(f"发送: {json.dumps(custom_msg, indent=2)}")
            
            # 监听消息几秒钟
            print("\n👂 监听服务器消息 (10秒)...")
            end_time = time.time() + 10
            
            while time.time() < end_time:
                try:
                    message = await asyncio.wait_for(websocket.recv(), timeout=1.0)
                    print(f"📨 收到消息: {message}")
                except asyncio.TimeoutError:
                    print(".", end="", flush=True)
                    continue
                except websockets.exceptions.ConnectionClosed:
                    print("\n❌ 连接被服务器关闭")
                    break
            
            print(f"\n✅ WebSocket测试完成!")
            
    except websockets.exceptions.ConnectionRefused:
        print("❌ 连接被拒绝，请确保服务器在运行")
    except Exception as e:
        print(f"❌ 连接错误: {e}")

if __name__ == "__main__":
    print("=" * 50)
    print("ROS Monitor WebSocket 客户端测试")
    print("=" * 50)
    
    asyncio.run(test_websocket())

