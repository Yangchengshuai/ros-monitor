#!/bin/bash

# ROS监控前端启动脚本

set -e

echo "=== ROS监控前端启动脚本 ==="

# 检查Node.js环境
if ! command -v node &> /dev/null; then
    echo "❌ Node.js未安装，请先安装Node.js"
    exit 1
fi

# 检查npm
if ! command -v npm &> /dev/null; then
    echo "❌ npm未安装，请先安装npm"
    exit 1
fi

echo "📱 Node.js版本: $(node --version)"
echo "📦 npm版本: $(npm --version)"

# 检查依赖
if [ ! -d "node_modules" ]; then
    echo "📥 安装前端依赖..."
    npm install
    echo "✅ 依赖安装完成"
else
    echo "✅ 依赖已安装"
fi

# 检查端口占用
if lsof -Pi :5173 -sTCP:LISTEN -t >/dev/null ; then
    echo "⚠️  端口5173已被占用，正在停止..."
    pkill -f "vite.*5173" || true
    sleep 2
fi

# 启动前端服务
echo "🚀 启动ROS监控前端服务..."
echo "   访问地址: http://localhost:5173"
echo "   按 Ctrl+C 停止服务"

npm run dev