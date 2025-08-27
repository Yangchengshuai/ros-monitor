#!/bin/bash

# 局域网ROS监控系统启动脚本
# 自动配置并启动前端和后端服务

echo "🚀 ROS监控系统局域网启动工具"
echo "================================"

# 获取当前目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# 检查是否为root用户
if [[ $EUID -eq 0 ]]; then
   echo "❌ 请不要使用root用户运行此脚本"
   exit 1
fi

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

# 检查依赖
check_dependencies() {
    print_status "检查系统依赖..."
    
    # 检查Node.js
    if ! command -v node &> /dev/null; then
        print_error "Node.js未安装，请先安装Node.js"
        exit 1
    fi
    
    # 检查npm
    if ! command -v npm &> /dev/null; then
        print_error "npm未安装，请先安装npm"
        exit 1
    fi
    
    # 检查Python
    if ! command -v python3 &> /dev/null; then
        print_error "Python3未安装，请先安装Python3"
        exit 1
    fi
    
    print_success "依赖检查完成"
}

# 获取网络配置
get_network_config() {
    print_status "检测网络配置..."
    
    # 获取服务器IP地址
    CURRENT_IP=$(hostname -I | awk '{print $1}')
    print_status "当前服务器IP地址: $CURRENT_IP"
    
    # 检查是否有多个IP地址
    IP_COUNT=$(hostname -I | wc -w)
    if [[ $IP_COUNT -gt 1 ]]; then
        print_warning "检测到多个网络接口，请选择要使用的IP地址："
        hostname -I | tr ' ' '\n' | nl
        read -p "请输入序号 (1-$IP_COUNT): " IP_CHOICE
        
        if [[ $IP_CHOICE -ge 1 && $IP_CHOICE -le $IP_COUNT ]]; then
            SELECTED_IP=$(hostname -I | tr ' ' '\n' | sed -n "${IP_CHOICE}p")
            print_success "已选择IP地址: $SELECTED_IP"
        else
            print_warning "无效选择，使用第一个IP地址: $CURRENT_IP"
            SELECTED_IP=$CURRENT_IP
        fi
    else
        SELECTED_IP=$CURRENT_IP
    fi
    
    # 端口配置
    BACKEND_PORT=8001
    FRONTEND_PORT=5173
    
    print_status "端口配置:"
    print_status "  后端API: $BACKEND_PORT"
    print_status "  前端服务: $FRONTEND_PORT"
    print_status "  WebSocket: $BACKEND_PORT"
}

# 配置环境变量
configure_environment() {
    print_status "配置环境变量..."
    
    ENV_FILE="env.local"
    
    # 备份原文件
    if [[ -f "$ENV_FILE" ]]; then
        cp "$ENV_FILE" "${ENV_FILE}.backup.$(date +%Y%m%d_%H%M%S)"
        print_status "已备份原配置文件"
    fi
    
    # 创建新的环境变量文件
    cat > "$ENV_FILE" << EOF
# 前端环境配置
# 开发环境 - 本地访问
VITE_API_BASE_URL=http://localhost:$BACKEND_PORT
VITE_WS_URL=ws://localhost:$BACKEND_PORT

# 局域网访问配置
VITE_LAN_IP=$SELECTED_IP

# 应用配置
VITE_APP_TITLE=ROS远程监控系统
VITE_APP_VERSION=1.0.0

# 开发配置
VITE_DEV_SERVER_HOST=0.0.0.0
VITE_DEV_SERVER_PORT=$FRONTEND_PORT
EOF
    
    print_success "环境变量配置完成"
}

# 检查端口占用
check_ports() {
    print_status "检查端口占用情况..."
    
    # 检查后端端口
    if netstat -tlnp 2>/dev/null | grep -q ":$BACKEND_PORT "; then
        print_warning "后端端口 $BACKEND_PORT 已被占用"
        netstat -tlnp | grep ":$BACKEND_PORT "
        read -p "是否要停止占用该端口的进程? (y/n): " stop_backend
        if [[ $stop_backend == "y" || $stop_backend == "Y" ]]; then
            sudo fuser -k $BACKEND_PORT/tcp
            print_success "已释放端口 $BACKEND_PORT"
        fi
    else
        print_success "后端端口 $BACKEND_PORT 可用"
    fi
    
    # 检查前端端口
    if netstat -tlnp 2>/dev/null | grep -q ":$FRONTEND_PORT "; then
        print_warning "前端端口 $FRONTEND_PORT 已被占用"
        netstat -tlnp | grep ":$FRONTEND_PORT "
        read -p "是否要停止占用该端口的进程? (y/n): " stop_frontend
        if [[ $stop_frontend == "y" || $stop_frontend == "Y" ]]; then
            sudo fuser -k $FRONTEND_PORT/tcp
            print_success "已释放端口 $FRONTEND_PORT"
        fi
    else
        print_success "前端端口 $FRONTEND_PORT 可用"
    fi
}

# 启动后端服务
start_backend() {
    print_status "启动后端服务..."
    
    # 检查后端目录
    BACKEND_DIR="../ros_monitor_backend"
    if [[ ! -d "$BACKEND_DIR" ]]; then
        print_error "后端目录不存在: $BACKEND_DIR"
        print_status "请确保在正确的目录中运行此脚本"
        return 1
    fi
    
    cd "$BACKEND_DIR"
    
    # 检查虚拟环境
    if [[ ! -d ".venv" ]]; then
        print_warning "虚拟环境不存在，正在创建..."
        python3 -m venv .venv
    fi
    
    # 激活虚拟环境
    source .venv/bin/activate
    
    # 安装依赖
    if [[ ! -f "requirements.txt" ]]; then
        print_error "requirements.txt不存在"
        return 1
    fi
    
    print_status "安装Python依赖..."
    pip install -r requirements.txt
    
    # 启动后端服务
    print_status "启动后端服务 (端口: $BACKEND_PORT)..."
    nohup uvicorn src.main:app --host 0.0.0.0 --port $BACKEND_PORT > backend.log 2>&1 &
    BACKEND_PID=$!
    
    # 等待服务启动
    sleep 3
    
    # 检查服务状态
    if curl -s "http://localhost:$BACKEND_PORT/api/v1/health" > /dev/null; then
        print_success "后端服务启动成功 (PID: $BACKEND_PID)"
        echo $BACKEND_PID > backend.pid
    else
        print_error "后端服务启动失败"
        return 1
    fi
    
    cd "$SCRIPT_DIR"
}

# 启动前端服务
start_frontend() {
    print_status "启动前端服务..."
    
    # 安装依赖
    if [[ ! -d "node_modules" ]]; then
        print_status "安装Node.js依赖..."
        npm install
    fi
    
    # 启动前端服务
    print_status "启动前端服务 (端口: $FRONTEND_PORT)..."
    nohup npm run dev > frontend.log 2>&1 &
    FRONTEND_PID=$!
    
    # 等待服务启动
    sleep 5
    
    # 检查服务状态
    if curl -s "http://localhost:$FRONTEND_PORT" > /dev/null; then
        print_success "前端服务启动成功 (PID: $FRONTEND_PID)"
        echo $FRONTEND_PID > frontend.pid
    else
        print_error "前端服务启动失败"
        return 1
    fi
}

# 检查防火墙
check_firewall() {
    print_status "检查防火墙设置..."
    
    if command -v ufw &> /dev/null; then
        print_status "检测到UFW防火墙"
        UFW_STATUS=$(sudo ufw status 2>/dev/null | grep -o "Status: .*")
        print_status "防火墙状态: $UFW_STATUS"
        
        if [[ $UFW_STATUS == *"inactive"* ]]; then
            print_warning "防火墙未启用，端口已开放"
        else
            print_status "检查端口 $BACKEND_PORT 和 $FRONTEND_PORT 是否开放..."
            
            if ! sudo ufw status | grep -q "$BACKEND_PORT"; then
                print_warning "端口 $BACKEND_PORT 未开放，正在开放..."
                sudo ufw allow $BACKEND_PORT
            fi
            
            if ! sudo ufw status | grep -q "$FRONTEND_PORT"; then
                print_warning "端口 $FRONTEND_PORT 未开放，正在开放..."
                sudo ufw allow $FRONTEND_PORT
            fi
            
            print_success "防火墙端口配置完成"
        fi
    elif command -v firewall-cmd &> /dev/null; then
        print_status "检测到firewalld防火墙"
        
        if ! sudo firewall-cmd --list-ports | grep -q "$BACKEND_PORT"; then
            print_warning "端口 $BACKEND_PORT 未开放，正在开放..."
            sudo firewall-cmd --permanent --add-port=$BACKEND_PORT/tcp
        fi
        
        if ! sudo firewall-cmd --list-ports | grep -q "$FRONTEND_PORT"; then
            print_warning "端口 $FRONTEND_PORT 未开放，正在开放..."
            sudo firewall-cmd --permanent --add-port=$FRONTEND_PORT/tcp
        fi
        
        sudo firewall-cmd --reload
        print_success "防火墙端口配置完成"
    else
        print_warning "未检测到防火墙，请手动检查端口是否开放"
    fi
}

# 显示系统信息
show_system_info() {
    print_success "系统启动完成！"
    echo ""
    echo "📋 系统信息:"
    echo "============"
    echo "服务器IP地址: $SELECTED_IP"
    echo "后端API端口: $BACKEND_PORT"
    echo "前端服务端口: $FRONTEND_PORT"
    echo "WebSocket地址: ws://$SELECTED_IP:$BACKEND_PORT"
    echo "前端访问地址: http://$SELECTED_IP:$FRONTEND_PORT"
    echo ""
    
    echo "🧪 测试命令:"
    echo "============"
    echo "# 测试后端API"
    echo "curl http://$SELECTED_IP:$BACKEND_PORT/api/v1/health"
    echo ""
    echo "# 测试前端访问"
    echo "curl http://$SELECTED_IP:$FRONTEND_PORT"
    echo ""
    echo "# 测试WebSocket连接"
    echo "ws://$SELECTED_IP:$BACKEND_PORT/ws/test_client"
    echo ""
    
    echo "📱 访问方式:"
    echo "============"
    echo "本地访问: http://localhost:$FRONTEND_PORT"
    echo "局域网访问: http://$SELECTED_IP:$FRONTEND_PORT"
    echo ""
    
    echo "📝 日志文件:"
    echo "============"
    echo "后端日志: ros_monitor_backend/backend.log"
    echo "前端日志: frontend.log"
    echo ""
    
    echo "🛑 停止服务:"
    echo "============"
    echo "停止后端: kill \$(cat ros_monitor_backend/backend.pid)"
    echo "停止前端: kill \$(cat frontend.pid)"
    echo "或者运行: ./stop_lan_system.sh"
}

# 主函数
main() {
    check_dependencies
    get_network_config
    configure_environment
    check_ports
    check_firewall
    
    # 启动服务
    if start_backend; then
        if start_frontend; then
            show_system_info
        else
            print_error "前端服务启动失败"
            exit 1
        fi
    else
        print_error "后端服务启动失败"
        exit 1
    fi
}

# 运行主函数
main







