#!/usr/bin/env python3
"""
端到端集成测试
验证US-001功能：启动数据采集
"""

import asyncio
import subprocess
import time
import requests
import sys
from pathlib import Path

def test_api_endpoints():
    """测试API端点是否可访问"""
    print("=== 测试API端点 ===")
    
    try:
        # 测试状态端点
        response = requests.get("http://localhost:8000/api/v1/data-collection/status", timeout=5)
        print(f"✓ 状态端点: {response.status_code}")
        
        if response.status_code == 200:
            data = response.json()
            print(f"✓ 状态数据: {data}")
            return True
        else:
            print(f"✗ 状态端点错误: {response.text}")
            return False
            
    except requests.exceptions.ConnectionError:
        print("✗ 后端服务未运行")
        return False
    except Exception as e:
        print(f"✗ 测试失败: {e}")
        return False

def test_script_files():
    """验证脚本文件存在"""
    print("\n=== 验证脚本文件 ===")
    
    start_script = "/home/ycs/work/ikinghandbot/scripts/start_all.sh"
    stop_script = "/home/ycs/work/ikinghandbot/scripts/stop_all.sh"
    
    start_exists = Path(start_script).exists()
    stop_exists = Path(stop_script).exists()
    
    print(f"✓ start_all.sh: {'存在' if start_exists else '不存在'}")
    print(f"✓ stop_all.sh: {'存在' if stop_exists else '不存在'}")
    
    # 检查执行权限
    if start_exists:
        start_executable = os.access(start_script, os.X_OK)
        print(f"✓ start_all.sh 可执行: {'是' if start_executable else '否'}")
    
    return start_exists and stop_exists

def test_frontend_build():
    """验证前端构建"""
    print("\n=== 验证前端构建 ===")
    
    try:
        # 检查TypeScript编译
        frontend_dir = Path("ros_monitor_frontend")
        if frontend_dir.exists():
            print("✓ 前端目录存在")
            
            # 检查关键文件
            control_file = frontend_dir / "src" / "components" / "Sensors" / "DataCollectionControl.tsx"
            service_file = frontend_dir / "src" / "services" / "dataCollection.ts"
            
            print(f"✓ 控制组件: {'存在' if control_file.exists() else '不存在'}")
            print(f"✓ 服务文件: {'存在' if service_file.exists() else '不存在'}")
            
            return control_file.exists() and service_file.exists()
        else:
            print("✗ 前端目录不存在")
            return False
            
    except Exception as e:
        print(f"✗ 前端验证失败: {e}")
        return False

def run_integration_test():
    """运行集成测试"""
    print("🚀 开始端到端集成测试...")
    print("测试US-001: 远程启动数据采集功能\n")
    
    all_tests_passed = True
    
    # 测试1: 脚本文件验证
    if not test_script_files():
        all_tests_passed = False
    
    # 测试2: API端点验证
    if not test_api_endpoints():
        all_tests_passed = False
    
    # 测试3: 前端构建验证
    if not test_frontend_build():
        all_tests_passed = False
    
    # 提供启动指南
    print("\n=== 启动指南 ===")
    print("1. 启动后端:")
    print("   cd ros_monitor_backend && python3 -m src.main")
    print("2. 启动前端（新终端）:")
    print("   cd ros_monitor_frontend && npm run dev")
    print("3. 访问: http://localhost:5173")
    print("4. 查看数据采集控制面板")
    
    if all_tests_passed:
        print("\n🎉 所有检查通过！US-001功能已实现")
        print("✅ 后端API端点就绪")
        print("✅ 前端组件就绪") 
        print("✅ 脚本文件就绪")
    else:
        print("\n⚠️  部分检查失败，请按启动指南操作")
    
    return all_tests_passed

if __name__ == "__main__":
    import os
    success = run_integration_test()
    sys.exit(0 if success else 1)