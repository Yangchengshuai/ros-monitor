#!/bin/bash

# 强制清理端口占用脚本

echo "🧹 强制清理端口占用脚本"
echo "=========================="

# 检查是否以root权限运行
if [ "$EUID" -ne 0 ]; then
    echo "⚠️  此脚本需要root权限来清理所有端口占用"
    echo "请使用: sudo $0"
    exit 1
fi

# 清理函数
cleanup_port() {
    local port=$1
    local service_name=$2
    
    echo "🔍 检查端口 $port ($service_name)..."
    
    if lsof -i :$port >/dev/null 2>&1; then
        echo "⚠️  端口 $port 被占用，正在强制清理..."
        
        # 获取所有占用端口的进程
        local pids=$(lsof -ti :$port)
        if [ -n "$pids" ]; then
            echo "   找到进程: $pids"
            
            for pid in $pids; do
                if kill -0 $pid 2>/dev/null; then
                    local process_info=$(ps -p $pid -o pid,ppid,cmd --no-headers 2>/dev/null)
                    echo "   进程 $pid: $process_info"
                    
                    # 强制停止进程
                    echo "   强制停止进程 $pid..."
                    kill -9 $pid 2>/dev/null
                    sleep 1
                    
                    # 验证是否停止
                    if kill -0 $pid 2>/dev/null; then
                        echo "   ❌ 无法停止进程 $pid"
                    else
                        echo "   ✅ 进程 $pid 已停止"
                    fi
                fi
            done
            
            # 等待端口释放
            echo "   ⏳ 等待端口 $port 释放..."
            local count=0
            while lsof -i :$port >/dev/null 2>&1 && [ $count -lt 10 ]; do
                echo "     等待中... ($count/10)"
                sleep 1
                count=$((count + 1))
            done
            
            if lsof -i :$port >/dev/null 2>&1; then
                echo "   ❌ 端口 $port 清理失败"
                return 1
            else
                echo "   ✅ 端口 $port 清理完成"
            fi
        fi
    else
        echo "✅ 端口 $port 可用"
    fi
    return 0
}

# 清理指定端口
echo "🚀 开始清理端口..."

# 清理后端端口
if cleanup_port 8000 "后端服务"; then
    echo "✅ 后端端口清理成功"
else
    echo "❌ 后端端口清理失败"
fi

# 清理前端端口
if cleanup_port 5173 "前端服务"; then
    echo "✅ 前端端口清理成功"
else
    echo "❌ 前端端口清理失败"
fi

# 清理可能的替代端口
echo ""
echo "🔍 清理可能的替代端口..."

for port in 8001 8002 8003 8004 8005; do
    if lsof -i :$port >/dev/null 2>&1; then
        echo "清理端口 $port..."
        cleanup_port $port "替代后端端口"
    fi
done

for port in 5174 5175 5176 5177 5178; do
    if lsof -i :$port >/dev/null 2>&1; then
        echo "清理端口 $port..."
        cleanup_port $port "替代前端端口"
    fi
done

echo ""
echo "🎯 端口清理完成！"
echo ""
echo "📋 当前端口状态:"
echo "端口 8000: $(lsof -i :8000 >/dev/null 2>&1 && echo "占用" || echo "可用")"
echo "端口 5173: $(lsof -i :5173 >/dev/null 2>&1 && echo "占用" || echo "可用")"

echo ""
echo "🚀 现在可以重新运行启动脚本:"
echo "  ./start_monitor_system.sh"
