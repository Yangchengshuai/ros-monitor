#!/bin/bash

# 后端相机数据接收测试启动脚本

echo "=========================================="
echo "启动后端相机数据接收测试"
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

# 检查话题是否存在
echo "检查相机话题..."
rostopic list | grep -E "(left_camera|right_camera)" | head -5

echo ""
echo "启动后端相机数据接收测试..."
echo "按 Ctrl+C 退出"
echo ""

# 运行测试器
cd /home/ycs/work/ikinghandbot/ros_monitor_backend
python3 test_backend_camera.py
