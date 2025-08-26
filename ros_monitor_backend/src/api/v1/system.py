from fastapi import APIRouter

router = APIRouter()

@router.get("/status")
def system_status():
    return {
        "success": True,
        "data": {
            "ros_master": {"connected": False, "uri": "http://localhost:11311", "topics": 0, "nodes": 0},
            "resources": {"cpu_usage": 0, "memory_usage": 0, "disk_usage": 0}
        }
    }
