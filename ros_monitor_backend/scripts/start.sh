#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
APP_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)

echo "=== 启动 ROS Monitor Backend ==="
echo "工作目录: $APP_ROOT"

# 设置Python路径
export PYTHONPATH="$APP_ROOT"

# 激活虚拟环境
if [ -f "$APP_ROOT/.venv/bin/activate" ]; then
  echo "激活虚拟环境..."
  source "$APP_ROOT/.venv/bin/activate"
else
  echo "警告: 未找到虚拟环境，使用系统Python"
fi

# 设置ROS环境
if [ -f "/opt/ros/noetic/setup.bash" ]; then
  echo "加载ROS环境..."
  source /opt/ros/noetic/setup.bash
else
  echo "警告: 未找到ROS环境"
fi

# 设置项目ROS环境
if [ -f "/home/ycs/work/ikinghandbot/devel/setup.bash" ]; then
  echo "加载项目ROS环境..."
  source /home/ycs/work/ikinghandbot/devel/setup.bash
fi

# 检查ROS Master
if ! pgrep -x "rosmaster" > /dev/null; then
  echo "警告: ROS Master未运行，请先启动: roscore"
fi

# 设置环境变量
export ROS_MASTER_URI=${ROS_MASTER_URI:-http://localhost:11311}
export ROS_HOSTNAME=${ROS_HOSTNAME:-localhost}

echo "ROS_MASTER_URI: $ROS_MASTER_URI"
echo "开始启动服务..."

cd "$APP_ROOT"
uvicorn src.main:app --host 0.0.0.0 --port 8000
