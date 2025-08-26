#!/bin/bash

# ==============================================================================
# 脚本名称: start_all.sh
# 功能描述: 
#   启动完整的LIVO系统，包括：
#   - lidar节点 (livox_ros_driver)
#   - camera节点 (mvs_ros_driver) 
#   - fastlivo算法节点
#   - rosbag录制节点
#
# 使用说明:
#   ./start_all.sh [bag_name]
#   例如: ./start_all.sh test_data
#   如果不指定bag_name，默认使用当前时间戳
#
# 注意事项:
#   1. 脚本启动前会检测 /tmp/livo_pids.txt 文件，若存在则退出，防止重复启动
#   2. 脚本会修改指定串口设备权限，请确保当前用户有 sudo 权限
#   3. 强烈建议在关闭之前的进程后再次运行脚本
#
# ==============================================================================

# 检查是否已有进程在运行
if [ -f "/tmp/livo_pids.txt" ]; then
  echo "检测到系统已在运行，请先执行 stop_all.sh 停止当前进程"
  exit 1
fi

# 获取bag文件名参数，如果未提供则使用时间戳，并规范化扩展名
RAW_BAG_NAME="$1"
if [ -z "$RAW_BAG_NAME" ]; then
  RAW_BAG_NAME="livo_$(date +%Y%m%d_%H%M%S)"
fi
# 去除可能的重复扩展名，如 .bag 或 .bag.active
BAG_BASE="${RAW_BAG_NAME%.bag}"
BAG_BASE="${BAG_BASE%.bag.active}"

echo "=== 启动LIVO系统 ==="
echo "Bag文件名: $BAG_BASE.bag"

# 设置设备权限
echo "设置设备权限..."
# 检查当前用户是否有权限访问设备，避免使用sudo
if [ -e /dev/ttyUSB0 ]; then
    ls -la /dev/ttyUSB0
    if [ -w /dev/ttyUSB0 ]; then
        echo "设备权限正常，无需修改"
    else
        echo "警告: /dev/ttyUSB0 没有写入权限，如果设备不可用请忽略"
    fi
else
    echo "信息: /dev/ttyUSB0 不存在，跳过权限设置"
fi

# 清空PID文件
> /tmp/livo_pids.txt

# 获取工作空间路径
WORKSPACE_PATH="/home/ycs/work/ikinghandbot"
DEVEL_PATH="$WORKSPACE_PATH/devel/setup.bash"

# 检查环境是否正确
if [ ! -f "$DEVEL_PATH" ]; then
  echo "错误: 找不到 $DEVEL_PATH"
  echo "请确保在正确的工作空间中运行脚本"
  exit 1
fi

# 启动命令列表
echo "启动各个节点..."

# 1. 启动lidar节点
echo "启动lidar节点..."
gnome-terminal --title="Lidar Node" -- bash -c "
  source $DEVEL_PATH
  roslaunch livox_ros_driver livox_lidar_msg.launch
  read -p 'Press Enter to close this terminal...'
" &
LIDAR_PID=$!
echo $LIDAR_PID >> /tmp/livo_pids.txt
sleep 3

# 2. 启动camera节点
echo "启动camera节点..."
gnome-terminal --title="Camera Node" -- bash -c "
  source $DEVEL_PATH
  LD_LIBRARY_PATH=/opt/MVS/lib/64:\$LD_LIBRARY_PATH roslaunch mvs_ros_driver mvs_camera_trigger.launch
  read -p 'Press Enter to close this terminal...'
" &
CAMERA_PID=$!
echo $CAMERA_PID >> /tmp/livo_pids.txt
sleep 3

# # 3. 启动fastlivo算法节点
# echo "启动fastlivo算法节点..."
# gnome-terminal --title="FastLIVO Node" -- bash -c "
#   export LD_LIBRARY_PATH=\"/usr/lib/x86_64-linux-gnu\"
#   source $DEVEL_PATH
#   roslaunch fast_livo mapping_avia.launch
#   read -p 'Press Enter to close this terminal...'
# " &
# FASTLIVO_PID=$!
# echo $FASTLIVO_PID >> /tmp/livo_pids.txt
# sleep 3

# 4. 启动rosbag录制节点
echo "启动rosbag录制节点..."
# 创建data目录（如果不存在）
DATA_DIR="/home/ycs/work/data"
mkdir -p $DATA_DIR

gnome-terminal --title="RosBag Record" -- bash -c "
  source $DEVEL_PATH
  cd $DATA_DIR
  # 后台启动并记录rosbag真实PID，然后等待其退出
  rosbag record /livox/lidar /livox/imu /left_camera/image -O $BAG_BASE &
  echo \$! > /tmp/rosbag_pid
  wait \$(cat /tmp/rosbag_pid 2>/dev/null)
" &
# 记录终端PID（用于后续收尾），同时单独保存rosbag真实PID
ROSBAG_TERM_PID=$!
echo $ROSBAG_TERM_PID >> /tmp/livo_pids.txt
echo $ROSBAG_TERM_PID > /tmp/rosbag_term_pid

echo ""
echo "=== 所有节点启动完成 ==="
echo "Lidar节点 PID: $LIDAR_PID"  
echo "Camera节点 PID: $CAMERA_PID"
echo "FastLIVO节点 PID: $FASTLIVO_PID"
echo "RosBag终端 PID: $ROSBAG_TERM_PID"
if [ -f "/tmp/rosbag_pid" ]; then
  echo "RosBag进程 PID: $(cat /tmp/rosbag_pid)"
fi
echo "Bag文件保存路径: $DATA_DIR/$BAG_BASE.bag"
echo ""
echo "使用 ./stop_all.sh 来停止所有节点"
echo "或者手动关闭各个终端窗口"
