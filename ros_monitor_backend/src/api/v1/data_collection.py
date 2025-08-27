"""
数据采集控制API端点
最小化实现，遵循YAGNI和KISS原则
"""

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from typing import Optional
import logging
import time

from src.services.script_executor import script_executor

logger = logging.getLogger(__name__)
router = APIRouter(prefix="/data-collection", tags=["data-collection"])

class StartRequest(BaseModel):
    """启动数据采集请求"""
    script: str = "start_all.sh"
    timeout: int = 30

class StopRequest(BaseModel):
    """停止数据采集请求"""
    force: bool = False

class DataCollectionStatus(BaseModel):
    """数据采集状态"""
    is_running: bool
    process_id: Optional[int] = None
    start_time: Optional[float] = None
    script_path: str = ""
    last_update: float

# 状态存储（内存中，重启后重置）
_status_cache = {
    "is_running": False,
    "start_time": None,
    "script_path": "",
    "last_update": 0
}

@router.post("/start")
async def start_data_collection(request: StartRequest):
    """启动数据采集"""
    try:
        # 检查是否已在运行
        if _status_cache["is_running"]:
            raise HTTPException(
                status_code=400, 
                detail="Data collection already running"
            )
        
        # 执行启动脚本
        result = await script_executor.execute_script(request.script, request.timeout)
        
        if result.success:
            # 更新状态
            _status_cache.update({
                "is_running": True,
                "start_time": time.time(),
                "script_path": f"script/{request.script}",
                "last_update": time.time()
            })
            
            logger.info(f"Data collection started: {request.script}")
            return {
                "success": True,
                "message": "数据采集已启动",
                "data": {
                    "process_id": result.exit_code,  # 简化：用exit_code代替真实PID
                    "start_time": _status_cache["start_time"],
                    "script_path": _status_cache["script_path"]
                }
            }
        else:
            raise HTTPException(
                status_code=500,
                detail=f"Failed to start data collection: {result.error_message}"
            )
            
    except Exception as e:
        logger.error(f"Error starting data collection: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@router.post("/stop")
async def stop_data_collection(request: StopRequest):
    """停止数据采集"""
    try:
        if not _status_cache["is_running"]:
            return {
                "success": True,
                "message": "Data collection is not running"
            }
        
        # 执行停止脚本
        result = await script_executor.execute_script("stop_all.sh", 30)
        
        if result.success:
            # 更新状态
            _status_cache.update({
                "is_running": False,
                "start_time": None,
                "script_path": "",
                "last_update": time.time()
            })
            
            logger.info("Data collection stopped")
            return {
                "success": True,
                "message": "数据采集已停止",
                "data": {
                    "stop_time": time.time(),
                    "duration": time.time() - (_status_cache.get("start_time") or 0)
                }
            }
        else:
            raise HTTPException(
                status_code=500,
                detail=f"Failed to stop data collection: {result.error_message}"
            )
            
    except Exception as e:
        logger.error(f"Error stopping data collection: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@router.get("/status")
async def get_collection_status():
    """获取数据采集状态"""
    return {
        "success": True,
        "data": DataCollectionStatus(
            is_running=_status_cache["is_running"],
            process_id=12345 if _status_cache["is_running"] else None,  # 简化实现
            start_time=_status_cache["start_time"],
            script_path=_status_cache["script_path"],
            last_update=_status_cache["last_update"]
        ).dict()
    }