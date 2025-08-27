"""
Script Executor Service
最小化实现，遵循YAGNI和KISS原则
"""

import asyncio
import os
import logging
from typing import Dict, Optional, NamedTuple
from dataclasses import dataclass

logger = logging.getLogger(__name__)

# 允许的脚本目录
ALLOWED_SCRIPT_DIR = "script"
ALLOWED_SCRIPTS = {"start_all.sh", "stop_all.sh"}

@dataclass
class ExecutionResult:
    """脚本执行结果"""
    success: bool
    exit_code: Optional[int] = None
    stdout: str = ""
    stderr: str = ""
    error_message: str = ""

class ScriptExecutor:
    """最小化脚本执行器"""
    
    def __init__(self):
        self.active_processes: Dict[str, asyncio.subprocess.Process] = {}
    
    def _validate_script_path(self, script_name: str) -> bool:
        """验证脚本路径合法性"""
        if script_name not in ALLOWED_SCRIPTS:
            return False
            
        # 获取项目根目录（从当前文件向上三级）
        current_dir = os.path.dirname(os.path.abspath(__file__))
        project_root = os.path.dirname(os.path.dirname(os.path.dirname(current_dir)))
        full_path = os.path.join(project_root, ALLOWED_SCRIPT_DIR, script_name)
        
        return os.path.isfile(full_path) and os.access(full_path, os.X_OK)
    
    def _check_duplicate_start(self, script_name: str) -> bool:
        """检查是否已在运行"""
        return script_name not in self.active_processes
    
    async def execute_script(self, script_name: str, timeout: int = 30) -> ExecutionResult:
        """
        执行指定脚本
        
        Args:
            script_name: 脚本文件名（start_all.sh 或 stop_all.sh）
            timeout: 超时时间（秒）
            
        Returns:
            ExecutionResult: 执行结果
        """
        try:
            # 验证脚本
            if not self._validate_script_path(script_name):
                return ExecutionResult(
                    success=False,
                    error_message=f"Invalid script: {script_name}"
                )
            
            # 防止重复启动（仅对start_all.sh）
            if script_name == "start_all.sh" and not self._check_duplicate_start(script_name):
                return ExecutionResult(
                    success=False,
                    error_message="Data collection already running"
                )
            
            # 构建完整路径
            current_dir = os.path.dirname(os.path.abspath(__file__))
            project_root = os.path.dirname(os.path.dirname(os.path.dirname(current_dir)))
            script_path = os.path.join(project_root, ALLOWED_SCRIPT_DIR, script_name)
            
            # 执行脚本
            logger.info(f"Executing script: {script_path}")
            
            process = await asyncio.create_subprocess_exec(
                script_path,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE
            )
            
            # 对start_all.sh记录进程，stop_all.sh不记录
            if script_name == "start_all.sh":
                self.active_processes[script_name] = process
            
            try:
                stdout, stderr = await asyncio.wait_for(
                    process.communicate(), 
                    timeout=timeout
                )
                
                exit_code = process.returncode
                success = exit_code == 0
                
                # 清理进程记录
                if script_name in self.active_processes:
                    del self.active_processes[script_name]
                
                return ExecutionResult(
                    success=success,
                    exit_code=exit_code,
                    stdout=stdout.decode('utf-8') if stdout else "",
                    stderr=stderr.decode('utf-8') if stderr else "",
                    error_message="" if success else f"Script failed with exit code {exit_code}"
                )
                
            except asyncio.TimeoutError:
                process.kill()
                await process.wait()
                
                if script_name in self.active_processes:
                    del self.active_processes[script_name]
                    
                return ExecutionResult(
                    success=False,
                    error_message=f"Script execution timeout after {timeout}s"
                )
                
        except Exception as e:
            logger.error(f"Script execution error: {e}")
            return ExecutionResult(
                success=False,
                error_message=str(e)
            )
    
    async def get_process_status(self, script_name: str) -> Dict[str, any]:
        """获取进程状态"""
        if script_name not in self.active_processes:
            return {"is_running": False, "process_id": None}
        
        process = self.active_processes[script_name]
        return {
            "is_running": process.returncode is None,
            "process_id": process.pid,
            "return_code": process.returncode
        }
    
    async def cleanup_process(self, script_name: str) -> bool:
        """清理僵尸进程"""
        if script_name in self.active_processes:
            process = self.active_processes[script_name]
            if process.returncode is None:  # 仍在运行
                process.kill()
                await process.wait()
            
            del self.active_processes[script_name]
            return True
        return False

# 全局单例
script_executor = ScriptExecutor()