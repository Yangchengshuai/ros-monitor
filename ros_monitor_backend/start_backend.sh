#!/bin/bash

# ROS监控后端启动脚本
# 需要在虚拟环境下运行

set -e

echo "=== ROS监控后端启动脚本 ==="

# 检查虚拟环境
if [ ! -d ".venv" ]; then
    echo "❌ 虚拟环境不存在，正在创建..."
    python3 -m venv .venv
    echo "✅ 虚拟环境创建完成"
fi

# 激活虚拟环境
echo "🔧 激活虚拟环境..."
source .venv/bin/activate

# 检查依赖
echo "📦 检查Python依赖..."
if ! pip show fastapi > /dev/null 2>&1; then
    echo "📥 安装Python依赖..."
    pip install -r requirements.txt
    echo "✅ 依赖安装完成"
else
    echo "✅ 依赖已安装"
fi

# 检查ROS环境
echo "🤖 检查ROS环境..."
if [ -z "$ROS_DISTRO" ]; then
    echo "⚠️  ROS环境未加载，尝试加载..."
    if [ -f "/opt/ros/noetic/setup.bash" ]; then
        source /opt/ros/noetic/setup.bash
        echo "✅ ROS Noetic环境已加载"
    elif [ -f "/opt/ros/melodic/setup.bash" ]; then
        source /opt/ros/melodic/setup.bash
        echo "✅ ROS Melodic环境已加载"
    else
        echo "❌ 未找到ROS环境，请手动加载"
        exit 1
    fi
else
    echo "✅ ROS环境已加载: $ROS_DISTRO"
fi

# 检查IKing Handbot工作空间
if [ -f "/home/ycs/work/ikinghandbot/devel/setup.bash" ]; then
    echo "🏠 加载IKing Handbot工作空间..."
    source /home/ycs/work/ikinghandbot/devel/setup.bash
    echo "✅ 工作空间已加载"
else
    echo "⚠️  IKing Handbot工作空间未找到，跳过..."
fi

# 设置环境变量
export PYTHONPATH="${PYTHONPATH}:$(pwd)"
export ROS_MASTER_URI=${ROS_MASTER_URI:-"http://localhost:11311"}
export ROS_HOSTNAME=${ROS_HOSTNAME:-"localhost"}

echo "🌐 环境配置:"
echo "  - ROS_MASTER_URI: $ROS_MASTER_URI"
echo "  - ROS_HOSTNAME: $ROS_HOSTNAME"
echo "  - PYTHONPATH: $PYTHONPATH"

# 检查端口占用
if lsof -Pi :8000 -sTCP:LISTEN -t >/dev/null ; then
    echo "⚠️  端口8000已被占用，正在停止..."
    pkill -f "uvicorn.*8000" || true
    sleep 2
fi

# 启动后端服务
echo "🚀 启动ROS监控后端服务..."
echo "   访问地址: http://localhost:8000"
echo "   API文档: http://localhost:8000/docs"
echo "   按 Ctrl+C 停止服务"

python -m src.main