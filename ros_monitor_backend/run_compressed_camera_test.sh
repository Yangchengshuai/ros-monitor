#!/bin/bash

# 压缩图像相机测试脚本启动器

echo "=== 启动压缩图像相机测试器 ==="
echo "此脚本将测试压缩图像数据接收和图像保存功能"
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

# 检查压缩图像话题
echo "检查压缩图像话题状态..."
rostopic info /left_camera/image/compressed

echo ""
echo "启动压缩图像相机测试器..."
echo "按 Ctrl+C 退出"
echo ""

# 运行测试脚本
python3 test_compressed_camera.py