#!/bin/bash

# ==============================================================================
# 脚本名称: stop_all.sh
# 功能描述:
#   停止所有由 start_all.sh 脚本启动的进程，包括：
#   - lidar节点
#   - camera节点  
#   - fastlivo算法节点
#   - rosbag录制节点
#
# 使用说明:
#   ./stop_all.sh
#
# 注意事项:
#   1. 脚本会读取 /tmp/livo_pids.txt 文件中的 PID 列表
#   2. 会强制终止所有进程，请确保重要数据已保存
#   3. 会额外查找并停止相关的ROS节点进程
#
# ==============================================================================

echo "=== 停止LIVO系统 ==="

PID_FILE="/tmp/livo_pids.txt"

# 函数：安全地终止进程
kill_process() {
    local pid=$1
    local name=$2
    
    if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
        echo "停止 $name (PID: $pid)..."
        
        # 首先尝试优雅地终止
        kill -TERM "$pid" 2>/dev/null
        sleep 2
        
        # 如果进程仍在运行，强制终止
        if kill -0 "$pid" 2>/dev/null; then
            echo "强制终止 $name (PID: $pid)..."
            kill -KILL "$pid" 2>/dev/null
        fi
        
        # 验证进程是否已终止
        if ! kill -0 "$pid" 2>/dev/null; then
            echo "✓ $name 已停止"
        else
            echo "✗ 无法停止 $name"
        fi
    else
        echo "进程 $name (PID: $pid) 未运行或已停止"
    fi
}

# 如果PID文件存在，读取并停止记录的进程
if [ -f "$PID_FILE" ]; then
    echo "从PID文件停止进程..."
    
    # 逆序读取PID文件（后启动的先停止）
    tac "$PID_FILE" | while IFS= read -r pid; do
        if [ -n "$pid" ] && [[ "$pid" =~ ^[0-9]+$ ]]; then
            kill_process "$pid" "记录的进程"
        fi
    done
    
    echo "清理PID文件..."
    rm -f "$PID_FILE"
else
    echo "未找到PID文件，尝试查找相关进程..."
fi

echo ""
echo "查找并停止相关ROS进程..."

# 停止rosbag record进程（优先使用保存的真实PID，尽可能优雅地关闭）
if [ -f "/tmp/rosbag_pid" ]; then
    ROSBAG_PID=$(cat /tmp/rosbag_pid)
    if [ -n "$ROSBAG_PID" ] && kill -0 "$ROSBAG_PID" 2>/dev/null; then
        echo "尝试通过rosnode kill优雅关闭rosbag..."
        if command -v rosnode >/dev/null 2>&1; then
            RNODES=$(rosnode list 2>/dev/null | grep -E '^/record_' || true)
            if [ -n "$RNODES" ]; then
                echo "$RNODES" | while read -r n; do rosnode kill "$n" 2>/dev/null || true; done
                for i in {1..10}; do
                    if ! kill -0 "$ROSBAG_PID" 2>/dev/null; then
                        echo "✓ rosbag已通过rosnode kill停止"
                        break
                    fi
                    sleep 1
                done
            fi
        fi

        # 如仍在运行，发送SIGINT到整个进程组，确保压缩子进程也收到信号
        if kill -0 "$ROSBAG_PID" 2>/dev/null; then
            PGID=$(ps -o pgid= -p "$ROSBAG_PID" | tr -d ' ')
            if [ -n "$PGID" ]; then
                echo "通过SIGINT关闭rosbag进程组 PGID: $PGID..."
                kill -INT -$PGID 2>/dev/null
            else
                echo "通过SIGINT关闭rosbag (PID: $ROSBAG_PID)..."
                kill -INT "$ROSBAG_PID" 2>/dev/null
            fi
        fi

        # 等待更长时间以便正确flush与索引写入
        for i in {1..30}; do
            if ! kill -0 "$ROSBAG_PID" 2>/dev/null; then
                echo "✓ rosbag录制已正确停止"
                break
            fi
            echo "等待rosbag正确关闭文件... ($i/30)"
            sleep 1
        done
        if kill -0 "$ROSBAG_PID" 2>/dev/null; then
            echo "rosbag进程未在预期时间内结束，强制终止..."
            kill -KILL "$ROSBAG_PID" 2>/dev/null
            echo "⚠️  强制终止可能导致bag文件损坏"
        fi
    else
        echo "未找到有效的rosbag PID或进程已退出"
    fi
    rm -f /tmp/rosbag_pid
else
    # 兜底：如果没有保存的PID，回退到通过命令行匹配的方式
    ROSBAG_PIDS=$(pgrep -f "rosbag record")
    if [ -n "$ROSBAG_PIDS" ]; then
        echo "发现rosbag进程: $ROSBAG_PIDS"
        for pid in $ROSBAG_PIDS; do
            if kill -0 "$pid" 2>/dev/null; then
                PGID=$(ps -o pgid= -p "$pid" | tr -d ' ')
                if [ -n "$PGID" ]; then
                    echo "通过SIGINT关闭rosbag进程组 PGID: $PGID..."
                    kill -INT -$PGID 2>/dev/null
                else
                    echo "正在优雅关闭rosbag录制 (PID: $pid)..."
                    kill -INT "$pid" 2>/dev/null
                fi
                for i in {1..30}; do
                    if ! kill -0 "$pid" 2>/dev/null; then
                        echo "✓ rosbag录制已正确停止"
                        break
                    fi
                    echo "等待rosbag正确关闭文件... ($i/30)"
                    sleep 1
                done
                if kill -0 "$pid" 2>/dev/null; then
                    echo "rosbag进程未在预期时间内结束，强制终止..."
                    kill -KILL "$pid" 2>/dev/null
                    echo "⚠️  强制终止可能导致bag文件损坏"
                fi
            fi
        done
    else
        echo "未发现rosbag录制进程"
    fi
fi

# 停止livox相关进程  
LIVOX_PIDS=$(pgrep -f "livox")
if [ -n "$LIVOX_PIDS" ]; then
    echo "发现livox相关进程: $LIVOX_PIDS"
    for pid in $LIVOX_PIDS; do
        kill_process "$pid" "livox节点"
    done
else
    echo "未发现livox节点进程"
fi

# 停止mvs camera相关进程
MVS_PIDS=$(pgrep -f "mvs")
if [ -n "$MVS_PIDS" ]; then
    echo "发现mvs camera相关进程: $MVS_PIDS"
    for pid in $MVS_PIDS; do
        kill_process "$pid" "mvs camera节点"
    done
else
    echo "未发现mvs camera节点进程"
fi

# 停止fastlivo相关进程
FASTLIVO_PIDS=$(pgrep -f "fastlivo_mapping\|fast_livo")
if [ -n "$FASTLIVO_PIDS" ]; then
    echo "发现fastlivo相关进程: $FASTLIVO_PIDS"
    for pid in $FASTLIVO_PIDS; do
        kill_process "$pid" "fastlivo算法节点"
    done
else
    echo "未发现fastlivo算法节点进程"
fi

# 停止rviz进程
RVIZ_PIDS=$(pgrep -f "rviz")
if [ -n "$RVIZ_PIDS" ]; then
    echo "发现rviz进程: $RVIZ_PIDS"
    for pid in $RVIZ_PIDS; do
        kill_process "$pid" "rviz"
    done
fi

echo ""
echo "=== 系统停止完成 ==="
echo ""

# 显示剩余的相关进程（如果有）
REMAINING_PROCESSES=$(pgrep -f "livox\|mvs\|fastlivo\|rosbag.*record" 2>/dev/null)
if [ -n "$REMAINING_PROCESSES" ]; then
    echo "警告：发现以下进程可能仍在运行："
    ps -p $REMAINING_PROCESSES -o pid,cmd --no-headers 2>/dev/null || true
    echo ""
    echo "如需手动终止，可使用: kill -9 <PID>"
else
    echo "✓ 所有相关进程已成功停止"
fi

echo "数据文件保存在: /home/ycs/work/data/"
