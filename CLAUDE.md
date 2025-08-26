# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Architecture Overview

This is a ROS (Robot Operating System) remote monitoring system with a Python FastAPI backend and React TypeScript frontend. The system provides real-time sensor data visualization and remote control capabilities.

### System Components

- **Backend**: FastAPI Python server (`ros_monitor_backend/`) - Handles ROS integration, WebSocket connections, and data streaming
- **Frontend**: React TypeScript application (`ros_monitor_frontend/`) - Provides real-time dashboards and control interfaces
- **ROS Bridge**: Connects to ROS topics for camera, LiDAR, and IMU data
- **WebSocket**: Real-time bidirectional communication between frontend and backend

## Key Technologies

- **Backend**: FastAPI, uvicorn, websockets, OpenCV, numpy, ROS (rospy)
- **Frontend**: React 19, TypeScript, Vite, Ant Design, Zustand, ECharts
- **Communication**: WebSocket for real-time data, REST API for control commands

## Development Commands

### Backend (Python)
```bash
# Setup
python3 -m venv .venv
source .venv/bin/activate
pip install -r ros_monitor_backend/requirements.txt

# Development
uvicorn src.main:app --host 0.0.0.0 --port 8000 --reload
./ros_monitor_backend/scripts/start.sh

# Testing
./ros_monitor_backend/run_backend_test.sh
curl http://localhost:8000/api/v1/health
```

### Frontend (TypeScript)
```bash
cd ros_monitor_frontend
npm install
npm run dev        # Development server
npm run build      # Production build
npm run lint       # Linting
npm run preview    # Preview build
```

### Full System
```bash
# Complete system startup
./start_monitor_system.sh

# The script handles:
# - ROS environment setup
# - Port management (8000 backend, 5173 frontend)
# - Service health checks
# - Network accessibility testing
```

## API Endpoints

- **Health Check**: `GET /api/v1/health` - System status and ROS connection
- **System Status**: `GET /api/v1/system/status` - Detailed system information
- **WebSocket**: `ws://localhost:8000/ws/{client_id}` - Real-time data streaming

## WebSocket Message Types

- `subscribe` - Subscribe to sensor topics (camera, lidar, imu)
- `unsubscribe` - Unsubscribe from topics
- `request_system_status` - Get current system status
- `ping/pong` - Connection heartbeat
- `camera_settings` - Update camera configuration

## Key Files

- **Backend Entry**: `ros_monitor_backend/src/main.py`
- **ROS Integration**: `ros_monitor_backend/src/ros_bridge/`
- **Frontend Entry**: `ros_monitor_frontend/src/main.tsx`
- **WebSocket Handler**: `ros_monitor_frontend/src/hooks/useWebSocket.ts`
- **State Management**: `ros_monitor_frontend/src/stores/`

## ROS Topics Monitored

- Camera data: Various camera topics
- LiDAR data: `/livox/lidar`
- IMU data: `/livox/imu`

## Environment Setup

Required ROS environment variables and paths:
```bash
source /opt/ros/noetic/setup.bash
source /path/to/workspace/devel/setup.bash
export PYTHONPATH=$PYTHONPATH:/opt/ros/noetic/lib/python3/dist-packages
```

## Port Configuration

- **Backend**: 8000 (configurable via ROS_MONITOR_BACKEND_PORT)
- **Frontend**: 5173 (configurable via ROS_MONITOR_FRONTEND_PORT)
- **WebSocket**: Same as backend port

## Testing Resources

- WebSocket test: `ros_monitor_backend/test_websocket.html`
- API test: `ros_monitor_frontend/test_api_connection.html`
- Camera test: Multiple test scripts in `ros_monitor_backend/`

## Common Issues

1. **ROS Import Errors**: Ensure ROS environment is sourced before running
2. **Port Conflicts**: Use `./start_monitor_system.sh` for automatic port management
3. **Camera Topics**: Verify ROS topics are publishing with `rostopic list`
4. **Network Access**: Check firewall settings for ports 8000 and 5173