#!/bin/bash

# 修复版本的ROS监控系统启动脚本
# 确保端口配置一致：后端8000，前端5173

# 获取脚本所在目录的绝对路径
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "🔧 修复版本ROS监控系统启动脚本"
echo "=================================="
echo "脚本目录: $SCRIPT_DIR"

# 端口配置 - 固定使用8000端口
BACKEND_PORT=8000
FRONTEND_PORT=5173

echo "端口配置:"
echo "  后端API: $BACKEND_PORT"
echo "  前端服务: $FRONTEND_PORT"
echo "  WebSocket: $BACKEND_PORT"

# 检查端口占用
echo "检查端口占用情况..."

# 检查后端端口
if netstat -tlnp 2>/dev/null | grep -q ":$BACKEND_PORT "; then
    echo "⚠️  后端端口 $BACKEND_PORT 已被占用"
    netstat -tlnp | grep ":$BACKEND_PORT "
    read -p "是否要停止占用该端口的进程? (y/n): " stop_backend
    if [[ $stop_backend == "y" || $stop_backend == "Y" ]]; then
        sudo fuser -k $BACKEND_PORT/tcp
        echo "✅ 已释放端口 $BACKEND_PORT"
    fi
else
    echo "✅ 后端端口 $BACKEND_PORT 可用"
fi

# 检查前端端口
if netstat -tlnp 2>/dev/null | grep -q ":$FRONTEND_PORT "; then
    echo "⚠️  前端端口 $FRONTEND_PORT 已被占用"
    netstat -tlnp | grep ":$FRONTEND_PORT "
    read -p "是否要停止占用该端口的进程? (y/n): " stop_frontend
    if [[ $stop_frontend == "y" || $stop_frontend == "Y" ]]; then
        sudo fuser -k $FRONTEND_PORT/tcp
        echo "✅ 已释放端口 $FRONTEND_PORT"
    fi
else
    echo "✅ 前端端口 $FRONTEND_PORT 可用"
fi

# 配置环境变量
echo "配置环境变量..."
ENV_FILE="env.local"

# 备份原文件
if [[ -f "$ENV_FILE" ]]; then
    cp "$ENV_FILE" "${ENV_FILE}.backup.$(date +%Y%m%d_%H%M%S)"
    echo "已备份原配置文件"
fi

# 创建新的环境变量文件，确保端口配置一致
cat > "$ENV_FILE" << ENVEOF
# 前端环境配置
# 开发环境 - 本地访问
VITE_API_BASE_URL=http://localhost:$BACKEND_PORT
VITE_WS_URL=ws://localhost:$BACKEND_PORT

# 局域网访问配置
VITE_LAN_IP=192.168.43.32

# 端口配置 - 确保与后端一致
VITE_API_PORT=$BACKEND_PORT
VITE_WS_PORT=$BACKEND_PORT

# 应用配置
VITE_APP_TITLE=ROS远程监控系统
VITE_APP_VERSION=1.0.0

# 开发配置
VITE_DEV_SERVER_HOST=0.0.0.0
VITE_DEV_SERVER_PORT=$FRONTEND_PORT
ENVEOF

echo "✅ 环境变量配置完成"
echo "已设置端口: $BACKEND_PORT"

# 启动后端服务
echo "启动后端服务..."
BACKEND_DIR="../ros_monitor_backend"

if [[ ! -d "$BACKEND_DIR" ]]; then
    echo "❌ 后端目录不存在: $BACKEND_DIR"
    exit 1
fi

cd "$BACKEND_DIR"

# 检查虚拟环境
if [[ ! -d ".venv" ]]; then
    echo "⚠️  虚拟环境不存在，正在创建..."
    python3 -m venv .venv
fi

# 激活虚拟环境
source .venv/bin/activate

# 安装依赖
if [[ ! -f "requirements.txt" ]]; then
    echo "❌ requirements.txt不存在"
    exit 1
fi

echo "安装Python依赖..."
pip install -r requirements.txt

# 启动后端服务 - 明确指定端口8000
echo "启动后端服务 (端口: $BACKEND_PORT)..."
nohup uvicorn src.main:app --host 0.0.0.0 --port $BACKEND_PORT > backend.log 2>&1 &
BACKEND_PID=$!

# 等待服务启动
sleep 5

# 检查服务状态
if curl -s "http://localhost:$BACKEND_PORT/api/v1/health" > /dev/null; then
    echo "✅ 后端服务启动成功 (PID: $BACKEND_PID)"
    echo $BACKEND_PID > backend.pid
else
    echo "❌ 后端服务启动失败"
    exit 1
fi

cd "$SCRIPT_DIR"

# 启动前端服务
echo "启动前端服务..."
npm install

echo "启动前端服务 (端口: $FRONTEND_PORT)..."
nohup npm run dev > frontend.log 2>&1 &
FRONTEND_PID=$!

# 等待服务启动
sleep 8

# 检查服务状态
if curl -s "http://localhost:$FRONTEND_PORT" > /dev/null; then
    echo "✅ 前端服务启动成功 (PID: $FRONTEND_PID)"
    echo $FRONTEND_PID > frontend.pid
else
    echo "❌ 前端服务启动失败"
    exit 1
fi

echo "✅ 系统启动完成！"
echo ""
echo "📋 系统信息:"
echo "============"
echo "后端API端口: $BACKEND_PORT"
echo "前端服务端口: $FRONTEND_PORT"
echo "WebSocket地址: ws://localhost:$BACKEND_PORT"
echo "前端访问地址: http://localhost:$FRONTEND_PORT"
echo ""
echo "🧪 测试命令:"
echo "curl http://localhost:$BACKEND_PORT/api/v1/health"
echo "curl http://localhost:$FRONTEND_PORT"
