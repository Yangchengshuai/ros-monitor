#!/bin/bash

# ROS监控系统停止脚本

echo "=== ROS监控系统停止脚本 ==="

# 获取端口配置（如果存在）
if [ -f ".ros_monitor_ports" ]; then
    echo "📋 读取端口配置..."
    source .ros_monitor_ports
    echo "   后端端口: ${BACKEND_PORT:-8000}"
    echo "   前端端口: ${FRONTEND_PORT:-5173}"
    echo "   后端PID: ${BACKEND_PID:-未知}"
    echo "   前端PID: ${FRONTEND_PID:-未知}"
else
    echo "⚠️  端口配置文件不存在，使用默认值"
    BACKEND_PORT=8000
    FRONTEND_PORT=5173
fi

# 停止后端服务
echo ""
echo "🛑 停止后端服务..."
if [ -n "$BACKEND_PID" ] && kill -0 $BACKEND_PID 2>/dev/null; then
    echo "   停止后端进程 (PID: $BACKEND_PID)..."
    kill $BACKEND_PID 2>/dev/null || true
    sleep 3
    
    # 检查是否真的停止了
    if kill -0 $BACKEND_PID 2>/dev/null; then
        echo "   强制停止后端进程..."
        kill -9 $BACKEND_PID 2>/dev/null || true
        sleep 1
    fi
    
    if kill -0 $BACKEND_PID 2>/dev/null; then
        echo "   ⚠️  后端进程仍在运行"
    else
        echo "   ✅ 后端进程已停止"
    fi
else
    echo "   ℹ️  后端进程未运行或PID无效"
fi

# 停止前端服务
echo ""
echo "🛑 停止前端服务..."
if [ -n "$FRONTEND_PID" ] && kill -0 $FRONTEND_PID 2>/dev/null; then
    echo "   停止前端进程 (PID: $FRONTEND_PID)..."
    kill $FRONTEND_PID 2>/dev/null || true
    sleep 3
    
    # 检查是否真的停止了
    if kill -0 $FRONTEND_PID 2>/dev/null; then
        echo "   强制停止前端进程..."
        kill -9 $FRONTEND_PID 2>/dev/null || true
        sleep 1
    fi
    
    if kill -0 $FRONTEND_PID 2>/dev/null; then
        echo "   ⚠️  前端进程仍在运行"
    else
        echo "   ✅ 前端进程已停止"
    fi
else
    echo "   ℹ️  前端进程未运行或PID无效"
fi

# 停止roscore（如果是由启动脚本启动的）
echo ""
echo "🛑 检查roscore状态..."
if [ -f "/tmp/roscore.pid" ]; then
    ROSCORE_PID=$(cat /tmp/roscore.pid)
    if [ -n "$ROSCORE_PID" ] && kill -0 $ROSCORE_PID 2>/dev/null; then
        echo "   停止roscore进程 (PID: $ROSCORE_PID)..."
        
        # 先尝试优雅停止
        kill $ROSCORE_PID 2>/dev/null || true
        sleep 3
        
        # 如果还在运行，强制停止
        if kill -0 $ROSCORE_PID 2>/dev/null; then
            echo "   强制停止roscore..."
            kill -9 $ROSCORE_PID 2>/dev/null || true
            sleep 1
        fi
        
        # 再次检查是否真的停止了
        if kill -0 $ROSCORE_PID 2>/dev/null; then
            echo "   ⚠️  roscore进程仍在运行，尝试其他方法..."
            # 使用pkill强制停止所有roscore相关进程
            pkill -f "roscore" 2>/dev/null || true
            pkill -f "rosmaster" 2>/dev/null || true
            sleep 2
        fi
        
        if kill -0 $ROSCORE_PID 2>/dev/null; then
            echo "   ❌ roscore停止失败"
        else
            echo "   ✅ roscore已停止"
        fi
    else
        echo "   ℹ️  roscore进程未运行或PID无效"
    fi
    
    # 清理roscore相关文件
    rm -f /tmp/roscore.pid /tmp/roscore.log
    echo "   🧹 roscore相关文件已清理"
else
    echo "   ℹ️  未找到roscore PID文件"
fi

# 额外清理：确保所有相关进程都被停止
echo ""
echo "🧹 清理所有相关进程..."
echo "   清理roscore相关进程..."
pkill -f "roscore" 2>/dev/null || true
pkill -f "rosmaster" 2>/dev/null || true
pkill -f "rosout" 2>/dev/null || true

echo "   清理ros_monitor相关进程..."
pkill -f "ros_monitor" 2>/dev/null || true
pkill -f "uvicorn.*ros_monitor" 2>/dev/null || true
pkill -f "vite.*ros_monitor" 2>/dev/null || true

# 等待进程完全停止
sleep 2

# 检查端口占用
echo ""
echo "🔍 检查端口占用..."
if lsof -Pi :$BACKEND_PORT -sTCP:LISTEN -t >/dev/null 2>&1; then
    echo "⚠️  端口 $BACKEND_PORT 仍被占用"
    lsof -Pi :$BACKEND_PORT -sTCP:LISTEN
else
    echo "✅ 端口 $BACKEND_PORT 已释放"
fi

if lsof -Pi :$FRONTEND_PORT -sTCP:LISTEN -t >/dev/null 2>&1; then
    echo "⚠️  端口 $FRONTEND_PORT 仍被占用"
    lsof -Pi :$FRONTEND_PORT -sTCP:LISTEN
else
    echo "✅ 端口 $FRONTEND_PORT 已释放"
fi

# 清理配置文件
echo ""
echo "🧹 清理配置文件..."
rm -f .ros_monitor_ports

echo ""
echo "✅ 系统停止完成！"
echo ""
echo "📝 如果仍有进程未停止，可以手动运行:"
echo "  pkill -f 'roscore'"
echo "  pkill -f 'ros_monitor'"
echo "  pkill -f 'uvicorn'"
echo "  pkill -f 'vite'"








