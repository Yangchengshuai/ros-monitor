#!/bin/bash

# 相机系统完整调试启动脚本

echo "=========================================="
echo "启动相机系统完整调试器"
echo "=========================================="

# 检查ROS环境
if [ -z "$ROS_DISTRO" ]; then
    echo "错误: ROS环境未设置，请先运行 source /opt/ros/noetic/setup.bash"
    exit 1
fi

# 检查工作空间
if [ ! -f "/home/ycs/work/ikinghandbot/devel/setup.bash" ]; then
    echo "错误: 工作空间未编译，请先运行 catkin_make"
    exit 1
fi

# 设置环境
source /home/ycs/work/ikinghandbot/devel/setup.bash

# 检查Python依赖
echo "检查Python依赖..."
python3 -c "import rospy, requests" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "错误: 缺少必要的Python依赖包"
    echo "请安装: pip3 install requests"
    exit 1
fi

# 检查话题是否存在
echo "检查相机话题..."
rostopic list | grep -E "(left_camera|right_camera)" | head -5

# 检查后端是否运行
echo "检查后端服务..."
if ! curl -s http://localhost:8000/api/v1/health > /dev/null; then
    echo "警告: 后端服务未运行，请先启动后端服务"
    echo "运行: ./start_backend.sh"
fi

echo ""
echo "启动相机系统完整调试器..."
echo "按 Ctrl+C 退出"
echo ""

# 运行调试器
cd /home/ycs/work/ikinghandbot/ros_monitor_backend
python3 debug_camera_system.py
