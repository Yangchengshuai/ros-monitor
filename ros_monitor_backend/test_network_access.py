#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
测试局域网访问
"""

import requests
import socket
import subprocess
import sys

def get_local_ip():
    """获取本机局域网IP"""
    try:
        # 连接到外部地址来获取本机IP
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        local_ip = s.getsockname()[0]
        s.close()
        return local_ip
    except Exception as e:
        print(f"获取本机IP失败: {e}")
        return None

def test_localhost():
    """测试localhost访问"""
    try:
        response = requests.get("http://localhost:8000/api/v1/health", timeout=5)
        print(f"✅ localhost访问成功: {response.status_code}")
        print(f"   响应内容: {response.text[:100]}...")
        return True
    except Exception as e:
        print(f"❌ localhost访问失败: {e}")
        return False

def test_local_ip(local_ip):
    """测试局域网IP访问"""
    try:
        response = requests.get(f"http://{local_ip}:8000/api/v1/health", timeout=5)
        print(f"✅ 局域网IP访问成功: {response.status_code}")
        print(f"   响应内容: {response.text[:100]}...")
        return True
    except Exception as e:
        print(f"❌ 局域网IP访问失败: {e}")
        return False

def test_network_connectivity():
    """测试网络连通性"""
    print("=== 网络连通性测试 ===")
    
    # 获取本机IP
    local_ip = get_local_ip()
    if local_ip:
        print(f"本机局域网IP: {local_ip}")
    else:
        print("无法获取本机IP")
        return
    
    # 测试localhost
    print("\n--- 测试localhost访问 ---")
    localhost_ok = test_localhost()
    
    # 测试局域网IP
    print(f"\n--- 测试局域网IP访问 ({local_ip}) ---")
    local_ip_ok = test_local_ip(local_ip)
    
    # 检查端口监听
    print("\n--- 检查端口监听 ---")
    try:
        result = subprocess.run(["netstat", "-tlnp"], capture_output=True, text=True)
        lines = result.stdout.split('\n')
        port_8000_lines = [line for line in lines if ':8000' in line]
        
        if port_8000_lines:
            print("✅ 端口8000正在监听:")
            for line in port_8000_lines:
                print(f"   {line.strip()}")
        else:
            print("❌ 端口8000未监听")
    except Exception as e:
        print(f"检查端口监听失败: {e}")
    
    # 总结
    print("\n=== 测试总结 ===")
    if localhost_ok and local_ip_ok:
        print("✅ 局域网访问正常")
    elif localhost_ok and not local_ip_ok:
        print("❌ 局域网访问失败，但localhost正常")
        print("   可能原因：防火墙、网络配置、应用绑定问题")
    else:
        print("❌ 所有访问都失败")

if __name__ == "__main__":
    test_network_connectivity()
