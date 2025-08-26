#!/bin/bash

# 独立的相机测试脚本启动器

echo "=== 启动独立的相机测试器 ==="
echo "此脚本将测试相机数据接收和图像保存功能"
echo ""

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

echo "ROS环境: $ROS_DISTRO"
echo "工作空间: /home/ycs/work/ikinghandbot"
echo ""

# 检查相机话题
echo "检查相机话题状态..."
rostopic list | grep -E "(camera|image)" || echo "未找到相机相关话题"

echo ""
echo "启动相机测试器..."
echo "按 Ctrl+C 退出"
echo ""

# 运行测试脚本
python3 test_camera_standalone.py