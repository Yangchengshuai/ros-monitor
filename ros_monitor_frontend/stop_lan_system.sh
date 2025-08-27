#!/bin/bash

# 停止局域网ROS监控系统脚本

echo "🛑 停止ROS监控系统"
echo "=================="

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 打印带颜色的消息
print_status() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# 停止前端服务
stop_frontend() {
    print_status "停止前端服务..."
    
    if [[ -f "frontend.pid" ]]; then
        FRONTEND_PID=$(cat frontend.pid)
        if kill -0 $FRONTEND_PID 2>/dev/null; then
            kill $FRONTEND_PID
            print_success "前端服务已停止 (PID: $FRONTEND_PID)"
        else
            print_warning "前端服务进程不存在 (PID: $FRONTEND_PID)"
        fi
        rm -f frontend.pid
    else
        print_warning "前端PID文件不存在"
    fi
    
    # 强制停止所有vite进程
    VITE_PIDS=$(pgrep -f "vite")
    if [[ -n "$VITE_PIDS" ]]; then
        print_status "强制停止所有vite进程..."
        echo $VITE_PIDS | xargs kill -9
        print_success "已停止所有vite进程"
    fi
}

# 停止后端服务
stop_backend() {
    print_status "停止后端服务..."
    
    BACKEND_DIR="../ros_monitor_backend"
    if [[ -d "$BACKEND_DIR" ]]; then
        cd "$BACKEND_DIR"
        
        if [[ -f "backend.pid" ]]; then
            BACKEND_PID=$(cat backend.pid)
            if kill -0 $BACKEND_PID 2>/dev/null; then
                kill $BACKEND_PID
                print_success "后端服务已停止 (PID: $BACKEND_PID)"
            else
                print_warning "后端服务进程不存在 (PID: $BACKEND_PID)"
            fi
            rm -f backend.pid
        else
            print_warning "后端PID文件不存在"
        fi
        
        cd - > /dev/null
    else
        print_warning "后端目录不存在: $BACKEND_DIR"
    fi
    
    # 强制停止所有uvicorn进程
    UVICORN_PIDS=$(pgrep -f "uvicorn")
    if [[ -n "$UVICORN_PIDS" ]]; then
        print_status "强制停止所有uvicorn进程..."
        echo $UVICORN_PIDS | xargs kill -9
        print_success "已停止所有uvicorn进程"
    fi
}

# 检查端口占用
check_ports() {
    print_status "检查端口占用情况..."
    
    BACKEND_PORT=8001
    FRONTEND_PORT=5173
    
    # 检查后端端口
    if netstat -tlnp 2>/dev/null | grep -q ":$BACKEND_PORT "; then
        print_warning "后端端口 $BACKEND_PORT 仍被占用:"
        netstat -tlnp | grep ":$BACKEND_PORT "
    else
        print_success "后端端口 $BACKEND_PORT 已释放"
    fi
    
    # 检查前端端口
    if netstat -tlnp 2>/dev/null | grep -q ":$FRONTEND_PORT "; then
        print_warning "前端端口 $FRONTEND_PORT 仍被占用:"
        netstat -tlnp | grep ":$FRONTEND_PORT "
    else
        print_success "前端端口 $FRONTEND_PORT 已释放"
    fi
}

# 清理临时文件
cleanup() {
    print_status "清理临时文件..."
    
    # 清理日志文件
    if [[ -f "frontend.log" ]]; then
        rm -f frontend.log
        print_status "已删除前端日志文件"
    fi
    
    if [[ -d "../ros_monitor_backend" ]] && [[ -f "../ros_monitor_backend/backend.log" ]]; then
        rm -f ../ros_monitor_backend/backend.log
        print_status "已删除后端日志文件"
    fi
    
    # 清理PID文件
    rm -f frontend.pid
    if [[ -d "../ros_monitor_backend" ]]; then
        rm -f ../ros_monitor_backend/backend.pid
    fi
    
    print_success "清理完成"
}

# 显示系统状态
show_status() {
    print_status "系统状态检查..."
    
    BACKEND_PORT=8001
    FRONTEND_PORT=5173
    
    echo ""
    echo "📋 当前状态:"
    echo "============"
    
    # 检查前端服务
    if pgrep -f "vite" > /dev/null; then
        echo "前端服务: 🔴 运行中"
    else
        echo "前端服务: 🟢 已停止"
    fi
    
    # 检查后端服务
    if pgrep -f "uvicorn" > /dev/null; then
        echo "后端服务: 🔴 运行中"
    else
        echo "后端服务: 🟢 已停止"
    fi
    
    # 检查端口占用
    if netstat -tlnp 2>/dev/null | grep -q ":$FRONTEND_PORT "; then
        echo "前端端口 $FRONTEND_PORT: 🔴 被占用"
    else
        echo "前端端口 $FRONTEND_PORT: 🟢 可用"
    fi
    
    if netstat -tlnp 2>/dev/null | grep -q ":$BACKEND_PORT "; then
        echo "后端端口 $BACKEND_PORT: 🔴 被占用"
    else
        echo "后端端口 $BACKEND_PORT: 🟢 可用"
    fi
}

# 主函数
main() {
    stop_frontend
    stop_backend
    check_ports
    cleanup
    show_status
    
    echo ""
    print_success "系统已停止！"
    echo ""
    echo "🔄 重新启动系统:"
    echo "  ./start_lan_system.sh"
    echo ""
    echo "📖 查看帮助文档:"
    echo "  cat README_LAN_ACCESS.md"
}

# 运行主函数
main







