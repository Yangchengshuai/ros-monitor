#!/bin/bash

# ROS监控系统完整启动脚本

set -e

echo "=== ROS监控系统启动脚本 ==="
echo "此脚本将启动完整的ROS监控系统"
echo ""

# 获取本机IP地址
get_local_ip() {
    # 优先获取实际的局域网IP地址
    local ip=$(ip route get 1.2.3.4 | awk '{print $7}' | head -n1)
    
    # 如果获取失败，尝试获取WiFi接口的IP
    if [ -z "$ip" ] || [[ "$ip" == "198.18.0.1" ]]; then
        ip=$(ip addr show wlx9c478242d544 2>/dev/null | grep 'inet ' | awk '{print $2}' | cut -d'/' -f1 | head -n1)
    fi
    
    # 如果还是失败，使用hostname
    if [ -z "$ip" ]; then
        ip=$(hostname -I | awk '{print $1}')
    fi
    
    echo "$ip"
}

LOCAL_IP=$(get_local_ip)
echo "🌐 本机IP地址: $LOCAL_IP"
echo ""

# 检查工作目录
if [ ! -f "ros_monitor_backend/start_backend.sh" ]; then
    echo "❌ 请在ros-monitor项目根目录运行此脚本"
    exit 1
fi

# 检查ROS环境
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
        echo "   运行: source /opt/ros/noetic/setup.bash"
        exit 1
    fi
else
    echo "✅ ROS环境已加载: $ROS_DISTRO"
fi

# 加载IKing Handbot工作空间
if [ -f "devel/setup.bash" ]; then
    echo "🏠 加载IKing Handbot工作空间..."
    source devel/setup.bash
    echo "✅ 工作空间已加载"
else
    echo "⚠️  工作空间未编译，尝试编译..."
    if [ -f "src/CMakeLists.txt" ]; then
        echo "🔨 编译工作空间..."
        catkin_make
        source devel/setup.bash
        echo "✅ 工作空间编译完成并加载"
    else
        echo "❌ 未找到CMakeLists.txt，跳过工作空间加载"
    fi
fi

# 检查ROS Master
echo "🔍 检查ROS Master状态..."
if ! rostopic list > /dev/null 2>&1; then
    echo "⚠️  ROS Master未运行，正在启动roscore..."
    
    # 启动roscore（后台运行）
    echo "🚀 启动roscore..."
    roscore > /tmp/roscore.log 2>&1 &
    ROSCORE_PID=$!
    
    # 等待roscore启动
    echo "⏳ 等待roscore启动..."
    local count=0
    local max_wait=30
    
    while [ $count -lt $max_wait ]; do
        if rostopic list > /dev/null 2>&1; then
            echo "✅ roscore启动成功 (PID: $ROSCORE_PID)"
            echo "   日志文件: /tmp/roscore.log"
            break
        fi
        
        echo "   等待中... ($count/$max_wait)"
        sleep 1
        count=$((count + 1))
    done
    
    if [ $count -eq $max_wait ]; then
        echo "❌ roscore启动超时"
        echo "检查日志: tail -f /tmp/roscore.log"
        kill $ROSCORE_PID 2>/dev/null || true
        exit 1
    fi
    
    # 保存roscore PID到文件，方便后续停止
    echo $ROSCORE_PID > /tmp/roscore.pid
    echo "💾 roscore PID已保存到 /tmp/roscore.pid"
    
else
    echo "✅ ROS Master运行正常"
fi

# 检查相机话题
echo "📷 检查相机话题..."
camera_topics=$(rostopic list | grep -E "(camera|image)" || true)
if [ -n "$camera_topics" ]; then
    echo "✅ 找到相机话题:"
    echo "$camera_topics" | sed 's/^/  - /'
else
    echo "⚠️  未找到相机话题，系统将使用测试数据"
fi

# 清理端口占用
echo "🧹 清理端口占用..."
cleanup_ports() {
    local port=$1
    local service_name=$2
    
    echo "🔍 检查端口 $port ($service_name)..."
    
    if lsof -Pi :$port -sTCP:LISTEN -t >/dev/null 2>&1; then
        echo "⚠️  端口 $port 被占用，正在分析..."
        
        # 获取占用端口的进程详细信息
        local port_info=$(lsof -Pi :$port -sTCP:LISTEN)
        echo "   端口占用信息:"
        echo "$port_info" | sed 's/^/      /'
        
        # 获取占用端口的进程PID
        local pids=$(lsof -ti :$port)
        if [ -n "$pids" ]; then
            echo "   找到进程PID: $pids"
            
            # 分析进程类型，智能决定是否清理
            for pid in $pids; do
                if kill -0 $pid 2>/dev/null; then
                    local process_info=$(ps -p $pid -o pid,ppid,cmd --no-headers 2>/dev/null)
                    echo "   进程 $pid 信息: $process_info"
                    
                    # 检查是否是我们的监控服务
                    if echo "$process_info" | grep -q "ros_monitor\|uvicorn\|fastapi"; then
                        echo "   🔄 检测到旧的监控服务进程，正在清理..."
                        local should_kill=true
                    elif echo "$process_info" | grep -q "python.*app.py"; then
                        echo "   🔄 检测到Python应用进程，正在清理..."
                        local should_kill=true
                    else
                        echo "   ⚠️  检测到未知进程，询问是否清理..."
                        read -p "   是否强制清理进程 $pid？(y/n): " -n 1 -r
                        echo
                        if [[ $REPLY =~ ^[Yy]$ ]]; then
                            local should_kill=true
                        else
                            echo "   ❌ 跳过清理，端口 $port 仍被占用"
                            return 1
                        fi
                    fi
                    
                    if [ "$should_kill" = true ]; then
                        echo "   正在停止进程 $pid..."
                        
                        # 尝试优雅停止
                        kill $pid
                        sleep 2
                        
                        # 如果还在运行，强制停止
                        if kill -0 $pid 2>/dev/null; then
                            echo "   进程仍在运行，强制停止..."
                            kill -9 $pid
                            sleep 1
                        fi
                        
                        # 验证进程是否已停止
                        if kill -0 $pid 2>/dev/null; then
                            echo "   ❌ 无法停止进程 $pid"
                            return 1
                        else
                            echo "   ✅ 进程 $pid 已停止"
                        fi
                    fi
                fi
            done
            
            # 等待端口释放
            echo "   ⏳ 等待端口 $port 释放..."
            local count=0
            local max_wait=15
            while lsof -Pi :$port -sTCP:LISTEN -t >/dev/null 2>&1 && [ $count -lt $max_wait ]; do
                echo "     等待中... ($count/$max_wait)"
                sleep 1
                count=$((count + 1))
            done
            
            if lsof -Pi :$port -sTCP:LISTEN -t >/dev/null 2>&1; then
                echo "   ❌ 端口 $port 清理失败，等待超时"
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

# 智能端口分配函数
find_available_port() {
    local base_port=$1
    local service_name=$2
    local max_attempts=10
    local current_port=$base_port
    
    for ((i=0; i<max_attempts; i++)); do
        if ! lsof -Pi :$current_port -sTCP:LISTEN -t >/dev/null 2>&1; then
            echo "✅ 找到可用端口: $current_port ($service_name)"
            return $current_port
        else
            echo "⚠️  端口 $current_port 被占用，尝试下一个..."
            current_port=$((current_port + 1))
        fi
    done
    
    echo "❌ 无法找到可用端口 (尝试范围: $base_port-$((base_port + max_attempts - 1)))"
    return 1
}

# 清理并分配端口
echo "🔧 端口配置检查..."

# 检查后端端口
if ! cleanup_ports 8000 "后端服务"; then
    echo "🔄 尝试查找替代端口..."
    if find_available_port 8001 "后端服务"; then
        BACKEND_PORT=$?
        echo "📝 后端服务将使用端口: $BACKEND_PORT"
        # 更新环境变量
        export ROS_MONITOR_BACKEND_PORT=$BACKEND_PORT
    else
        echo "❌ 无法找到可用的后端端口"
        exit 1
    fi
else
    BACKEND_PORT=8000
    echo "✅ 后端服务使用默认端口: $BACKEND_PORT"
fi

# 检查前端端口
if ! cleanup_ports 5173 "前端服务"; then
    echo "🔄 尝试查找替代端口..."
    if find_available_port 5174 "前端服务"; then
        FRONTEND_PORT=$?
        echo "📝 前端服务将使用端口: $FRONTEND_PORT"
        # 更新环境变量
        export ROS_MONITOR_FRONTEND_PORT=$FRONTEND_PORT
    else
        echo "❌ 无法找到可用的前端端口"
        exit 1
    fi
else
    FRONTEND_PORT=5173
    echo "✅ 前端服务使用默认端口: $FRONTEND_PORT"
fi

echo ""
echo "🚀 启动监控系统..."
echo "📋 端口配置:"
echo "  - 后端服务: $BACKEND_PORT"
echo "  - 前端服务: $FRONTEND_PORT"
echo ""

# 启动后端服务（后台运行）
echo "1️⃣ 启动后端服务..."
cd ros_monitor_backend
chmod +x start_backend.sh

# 传递端口参数给后端启动脚本
if [ "$BACKEND_PORT" != "8000" ]; then
    echo "📝 使用自定义端口: $BACKEND_PORT"
    # 创建临时启动脚本，包含端口配置
    cat > start_backend_temp.sh << EOF
#!/bin/bash
export ROS_MONITOR_BACKEND_PORT=$BACKEND_PORT
export ROS_MONITOR_HOST=0.0.0.0
./start_backend.sh
EOF
    chmod +x start_backend_temp.sh
    ./start_backend_temp.sh &
    BACKEND_PID=$!
    # 清理临时文件
    rm -f start_backend_temp.sh
else
    ./start_backend.sh &
    BACKEND_PID=$!
fi

cd ..

# 等待后端启动
echo "⏳ 等待后端服务启动..."
sleep 8

# 检查后端状态
echo "🔍 检查后端服务状态..."
max_retries=5
retry_count=0
backend_ready=false

while [ $retry_count -lt $max_retries ] && [ "$backend_ready" = false ]; do
    echo "⏳ 尝试连接后端服务... ($((retry_count + 1))/$max_retries)"
    
    # 尝试连接并检查响应内容
    response=$(curl -s -m 10 http://localhost:$BACKEND_PORT/api/v1/health 2>/dev/null)
    exit_code=$?
    
    if [ $exit_code -eq 0 ] && [ -n "$response" ]; then
        # 检查响应是否包含预期的JSON格式
        if echo "$response" | grep -q '"success"' || echo "$response" | grep -q '"ros_ready"'; then
            backend_ready=true
            echo "✅ 后端服务启动成功 (端口: $BACKEND_PORT)"
            echo "   响应: $response"
        else
            echo "⚠️  端口 $BACKEND_PORT 有服务响应，但格式不正确"
            echo "   响应: $response"
            echo "   可能是其他服务占用了端口"
            
            # 尝试查找占用端口的进程
            echo "🔍 检查端口占用..."
            if command -v ss >/dev/null 2>&1; then
                ss -tlnp | grep ":$BACKEND_PORT " || echo "   无法获取详细信息"
            fi
            
            # 询问是否继续
            read -p "   是否继续等待后端服务启动？(y/n): " -n 1 -r
            echo
            if [[ ! $REPLY =~ ^[Yy]$ ]]; then
                echo "❌ 用户取消，停止启动"
                kill $BACKEND_PID 2>/dev/null || true
                exit 1
            fi
        fi
    else
        retry_count=$((retry_count + 1))
        echo "⏳ 等待后端服务响应... ($retry_count/$max_retries)"
        echo "   连接失败，退出码: $exit_code"
        sleep 2
    fi
done

if [ "$backend_ready" = false ]; then
    echo "❌ 后端服务启动失败"
    echo "检查后端日志..."
    if [ -n "$BACKEND_PID" ] && kill -0 $BACKEND_PID 2>/dev/null; then
        echo "后端进程仍在运行，PID: $BACKEND_PID"
        echo "尝试获取进程信息..."
        ps -p $BACKEND_PID -o pid,ppid,cmd --no-headers 2>/dev/null || true
        
        # 检查进程是否真的在监听端口
        echo "🔍 检查进程端口绑定..."
        if command -v netstat >/dev/null 2>&1; then
            netstat -tlnp 2>/dev/null | grep "$BACKEND_PID" || echo "   进程未绑定到端口"
        fi
    fi
    kill $BACKEND_PID 2>/dev/null || true
    exit 1
fi

# 启动前端服务（后台运行）
echo "2️⃣ 启动前端服务..."
cd ros_monitor_frontend
chmod +x start_frontend.sh

# 传递端口参数给前端启动脚本
if [ "$FRONTEND_PORT" != "5173" ]; then
    echo "📝 使用自定义端口: $FRONTEND_PORT"
    # 创建临时启动脚本，包含端口配置
    cat > start_frontend_temp.sh << EOF
#!/bin/bash
export VITE_PORT=$FRONTEND_PORT
export VITE_BACKEND_URL=http://localhost:$BACKEND_PORT
./start_frontend.sh
EOF
    chmod +x start_frontend_temp.sh
    ./start_frontend_temp.sh &
    FRONTEND_PID=$!
    # 清理临时文件
    rm -f start_frontend_temp.sh
else
    ./start_frontend.sh &
    FRONTEND_PID=$!
fi

cd ..

# 等待前端启动
echo "⏳ 等待前端服务启动..."
sleep 5

# 检查前端状态
echo "🔍 检查前端服务状态..."
max_retries=5
retry_count=0
frontend_ready=false

while [ $retry_count -lt $max_retries ] && [ "$frontend_ready" = false ]; do
    if curl -s http://localhost:$FRONTEND_PORT > /dev/null 2>&1; then
        frontend_ready=true
        echo "✅ 前端服务启动成功 (端口: $FRONTEND_PORT)"
    else
        retry_count=$((retry_count + 1))
        echo "⏳ 等待前端服务响应... ($retry_count/$max_retries)"
        sleep 2
    fi
done

if [ "$frontend_ready" = false ]; then
    echo "⚠️  前端服务可能未完全启动，请检查..."
    if [ -n "$FRONTEND_PID" ] && kill -0 $FRONTEND_PID 2>/dev/null; then
        echo "前端进程仍在运行，PID: $FRONTEND_PID"
        echo "尝试获取进程信息..."
        ps -p $FRONTEND_PID -o pid,ppid,cmd --no-headers 2>/dev/null || true
    fi
fi

echo ""
echo "🎉 ROS监控系统启动完成！"
echo ""
echo "📱 本地访问:"
echo "  - 前端界面: http://localhost:$FRONTEND_PORT"
echo "  - 后端API: http://localhost:$BACKEND_PORT"
echo "  - API文档: http://localhost:$BACKEND_PORT/docs"
echo ""
echo "🌐 局域网访问:"
echo "  - 前端界面: http://$LOCAL_IP:$FRONTEND_PORT"
echo "  - 后端API: http://$LOCAL_IP:$BACKEND_PORT"
echo "  - API文档: http://$LOCAL_IP:$BACKEND_PORT/docs"
echo ""
echo "📋 系统状态:"
echo "  - 后端服务: 运行中 (PID: $BACKEND_PID)"
echo "  - 前端服务: 运行中 (PID: $FRONTEND_PID)"
echo ""

# 网络连接测试
echo "🔍 网络连接测试..."
echo "测试本地连接..."
if curl -s -m 5 http://localhost:$FRONTEND_PORT > /dev/null 2>&1; then
    echo "✅ 本地前端连接正常"
else
    echo "❌ 本地前端连接失败"
fi

if curl -s -m 5 http://localhost:$BACKEND_PORT/api/v1/health > /dev/null 2>&1; then
    echo "✅ 本地后端连接正常"
else
    echo "❌ 本地后端连接失败"
fi

echo ""
echo "测试局域网连接..."
if curl -s -m 5 http://$LOCAL_IP:$FRONTEND_PORT > /dev/null 2>&1; then
    echo "✅ 局域网前端连接正常"
else
    echo "❌ 局域网前端连接失败"
fi

if curl -s -m 5 http://$LOCAL_IP:$BACKEND_PORT/api/v1/health > /dev/null 2>&1; then
    echo "✅ 局域网后端连接正常"
else
    echo "❌ 局域网后端连接失败"
fi

echo ""
echo "🛑 停止系统:"
echo "  kill $BACKEND_PID $FRONTEND_PID"
echo "  或者运行: pkill -f 'ros_monitor'"
echo ""

# 保存端口配置到文件，方便后续使用
echo "💾 保存端口配置..."
cat > .ros_monitor_ports << EOF
# ROS监控系统端口配置
# 生成时间: $(date)
BACKEND_PORT=$BACKEND_PORT
FRONTEND_PORT=$FRONTEND_PORT
BACKEND_PID=$BACKEND_PID
FRONTEND_PID=$FRONTEND_PID
LOCAL_IP=$LOCAL_IP
EOF
echo "✅ 端口配置已保存到 .ros_monitor_ports"

# 显示快速访问命令
echo ""
echo "🚀 快速访问命令:"
echo "  # 查看端口配置"
echo "  cat .ros_monitor_ports"
echo ""
echo "  # 检查服务状态"
echo "  curl http://localhost:$BACKEND_PORT/api/v1/health"
echo "  curl http://localhost:$FRONTEND_PORT"
echo ""
echo "  # 查看进程状态"
echo "  ps aux | grep -E '($BACKEND_PID|$FRONTEND_PID)' | grep -v grep"
echo ""

# 等待用户输入
echo "按 Enter 键停止所有服务..."
read

# 停止服务
echo "🛑 停止监控系统..."
kill $BACKEND_PID $FRONTEND_PID 2>/dev/null || true

# 停止roscore（如果是由脚本启动的）
if [ -f "/tmp/roscore.pid" ]; then
    ROSCORE_PID=$(cat /tmp/roscore.pid)
    if [ -n "$ROSCORE_PID" ] && kill -0 $ROSCORE_PID 2>/dev/null; then
        echo "🛑 停止roscore (PID: $ROSCORE_PID)..."
        
        # 先尝试优雅停止
        kill $ROSCORE_PID 2>/dev/null || true
        sleep 3
        
        # 如果还在运行，强制停止
        if kill -0 $ROSCORE_PID 2>/dev/null; then
            echo "强制停止roscore..."
            kill -9 $ROSCORE_PID 2>/dev/null || true
            sleep 1
        fi
        
        # 再次检查是否真的停止了
        if kill -0 $ROSCORE_PID 2>/dev/null; then
            echo "⚠️  roscore进程仍在运行，尝试其他方法..."
            # 使用pkill强制停止所有roscore相关进程
            pkill -f "roscore" 2>/dev/null || true
            pkill -f "rosmaster" 2>/dev/null || true
            sleep 2
        fi
        
        echo "✅ roscore已停止"
    fi
    
    # 清理roscore相关文件
    rm -f /tmp/roscore.pid /tmp/roscore.log
    echo "🧹 roscore相关文件已清理"
fi

# 额外清理：确保所有ROS相关进程都被停止
echo "🧹 清理所有ROS相关进程..."
pkill -f "roscore" 2>/dev/null || true
pkill -f "rosmaster" 2>/dev/null || true
pkill -f "rosout" 2>/dev/null || true
sleep 1

# 等待进程完全停止
sleep 2

# 检查进程是否还在运行
if [ -n "$BACKEND_PID" ] && kill -0 $BACKEND_PID 2>/dev/null; then
    echo "强制停止后端服务..."
    kill -9 $BACKEND_PID 2>/dev/null || true
    sleep 1
fi

if [ -n "$FRONTEND_PID" ] && kill -0 $FRONTEND_PID 2>/dev/null; then
    echo "强制停止前端服务..."
    kill -9 $FRONTEND_PID 2>/dev/null || true
    sleep 1
fi

# 额外清理：确保所有相关进程都被停止
echo "🧹 清理所有相关进程..."
pkill -f "ros_monitor" 2>/dev/null || true
pkill -f "uvicorn.*ros_monitor" 2>/dev/null || true
pkill -f "vite.*ros_monitor" 2>/dev/null || true
sleep 2

# 清理端口配置文件
rm -f .ros_monitor_ports

echo "✅ 系统已停止"
echo ""
echo "📝 提示: 如需重新启动，请运行: ./start_monitor_system.sh"