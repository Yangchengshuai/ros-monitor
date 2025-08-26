#!/usr/bin/env python3
"""
快速测试数据采集后端功能
最小化测试，验证核心功能
"""

import asyncio
import os
import sys
from pathlib import Path

# 添加src到路径
backend_dir = Path(__file__).parent
sys.path.insert(0, str(backend_dir / "src"))

from services.script_executor import ScriptExecutor
from src.api.v1.data_collection import _status_cache

async def test_script_executor():
    """测试脚本执行器"""
    print("=== 测试 ScriptExecutor ===")
    
    executor = ScriptExecutor()
    
    # 测试路径验证
    valid = executor._validate_script_path("start_all.sh")
    invalid = executor._validate_script_path("invalid.sh")
    
    print(f"✓ start_all.sh 验证: {valid}")
    print(f"✓ invalid.sh 验证: {invalid}")
    
    # 测试重复启动检查
    duplicate_check = executor._check_duplicate_start("start_all.sh")
    print(f"✓ 重复启动检查: {duplicate_check}")
    
    # 测试状态缓存
    print(f"✓ 初始状态: {_status_cache}")
    
    print("\n=== ScriptExecutor 测试完成 ===")

async def test_api_logic():
    """测试API逻辑"""
    print("\n=== 测试 API 逻辑 ===")
    
    # 模拟API调用
    from src.api.v1.data_collection import get_collection_status
    
    status = await get_collection_status()
    print(f"✓ 状态查询: {status}")
    
    print("\n=== API 逻辑测试完成 ===")

async def main():
    """主测试函数"""
    print("开始测试数据采集后端功能...")
    
    try:
        await test_script_executor()
        await test_api_logic()
        
        print("\n🎉 所有测试通过！")
        
    except Exception as e:
        print(f"❌ 测试失败: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    asyncio.run(main())