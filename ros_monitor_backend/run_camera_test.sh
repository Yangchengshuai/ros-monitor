#!/bin/bash

# 相机测试启动脚本
# 用于测试BayerGB12Packed格式的相机数据接收

echo "=========================================="
echo "启动相机数据接收测试"
echo "=========================================="

# 检查ROS环境
if [ -z "$ROS_DISTRO" ]; then
    echo "错误: ROS环境未设置，请先运行 source /opt/ros/noetic/setup.bash"
    exit 1
fi

echo "ROS版本: $ROS_DISTRO"

# 检查工作空间
if [ ! -f "/home/ycs/work/ikinghandbot/devel/setup.bash" ]; then
    echo "错误: 工作空间未编译，请先运行 catkin_make"
    exit 1
fi

# 设置工作空间环境
source /home/ycs/work/ikinghandbot/devel/setup.bash
echo "工作空间环境已设置"

# 检查相机话题
echo "检查相机话题状态..."
rostopic list | grep -E "(camera|image)" || echo "警告: 未找到相机相关话题"

# 检查相机驱动状态
echo "检查相机驱动状态..."
ps aux | grep -E "(mvs_ros_driver|camera)" | grep -v grep || echo "警告: 未找到相机驱动进程"

# 启动测试
echo "启动相机测试器..."
cd /home/ycs/work/ikinghandbot/ros_monitor_backend

# 检查Python环境
if [ -d ".venv" ]; then
    echo "激活虚拟环境..."
    source .venv/bin/activate
fi

# 检查依赖
echo "检查Python依赖..."
python3 -c "import rospy, cv2, numpy" 2>/dev/null || {
    echo "错误: 缺少必要的Python依赖"
    echo "请安装: pip install opencv-python numpy"
    exit 1
}

# 运行测试
echo "运行相机测试器..."
python3 test_camera_simple.py

echo "相机测试完成"

