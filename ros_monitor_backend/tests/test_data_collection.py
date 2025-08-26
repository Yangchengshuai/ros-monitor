"""
US-001: 数据采集启动功能测试
最小测试集设计，遵循YAGNI和KISS原则
"""

import pytest
import asyncio
import os
from unittest.mock import patch, AsyncMock

# 测试用例：验证脚本执行功能
class TestDataCollectionStart:
    
    def test_script_path_validation(self):
        """测试脚本路径验证 - 只允许指定目录"""
        from src.services.script_executor import ScriptExecutor
        
        executor = ScriptExecutor()
        
        # 有效路径
        valid_path = "/home/ycs/work/ikinghandbot/scripts/start_all.sh"
        assert executor._validate_script_path(valid_path) == True
        
        # 无效路径 - 目录外
        invalid_path = "/etc/passwd"
        assert executor._validate_script_path(invalid_path) == False
        
    @pytest.mark.asyncio
    async def test_script_execution_success(self):
        """测试脚本成功执行"""
        from src.services.script_executor import ScriptExecutor
        
        executor = ScriptExecutor()
        
        # 使用mock避免真实执行
        with patch('asyncio.create_subprocess_exec') as mock_exec:
            mock_process = AsyncMock()
            mock_process.communicate.return_value = (b'success', b'')
            mock_process.returncode = 0
            mock_exec.return_value = mock_process
            
            result = await executor.execute_script("start_all.sh")
            assert result.success == True
            assert result.exit_code == 0
            
    @pytest.mark.asyncio 
    async def test_script_execution_failure(self):
        """测试脚本执行失败处理"""
        from src.services.script_executor import ScriptExecutor
        
        executor = ScriptExecutor()
        
        with patch('asyncio.create_subprocess_exec') as mock_exec:
            mock_process = AsyncMock()
            mock_process.communicate.return_value = (b'', b'error')
            mock_process.returncode = 1
            mock_exec.return_value = mock_process
            
            result = await executor.execute_script("start_all.sh")
            assert result.success == False
            assert result.exit_code == 1
            assert "error" in result.error_message

    def test_concurrent_execution_prevention(self):
        """测试防止重复启动"""
        from src.services.script_executor import ScriptExecutor
        
        executor = ScriptExecutor()
        executor.active_processes["start_all.sh"] = "mock_process"
        
        result = executor._check_duplicate_start("start_all.sh")
        assert result == False  # 应该拒绝重复启动

# 测试用例：API端点测试
class TestDataCollectionAPI:
    
    @pytest.mark.asyncio
    async def test_start_collection_endpoint(self, client):
        """测试启动采集API端点"""
        with patch('src.services.script_executor.ScriptExecutor.execute_script') as mock_execute:
            mock_execute.return_value.success = True
            
            response = await client.post("/api/v1/data-collection/start")
            assert response.status_code == 200
            assert response.json()["success"] == True
            assert "数据采集已启动" in response.json()["message"]

    @pytest.mark.asyncio
    async def test_status_endpoint(self, client):
        """测试状态查询端点"""
        response = await client.get("/api/v1/data-collection/status")
        assert response.status_code == 200
        assert "is_running" in response.json()["data"]
        assert isinstance(response.json()["data"]["is_running"], bool)

# 集成测试用例
class TestEndToEnd:
    
    @pytest.mark.asyncio
    async def test_full_start_stop_cycle(self):
        """测试完整的启动停止流程"""
        # 1. 检查初始状态
        # 2. 启动采集
        # 3. 验证状态更新
        # 4. 停止采集
        # 5. 验证最终状态
        pass  # 将在集成阶段实现

if __name__ == "__main__":
    pytest.main([__file__, "-v"])