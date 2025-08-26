#!/bin/bash

# 修复ROS环境变量脚本

echo "=========================================="
echo "修复ROS环境变量"
echo "=========================================="

# 设置ROS环境变量
export ROS_HOSTNAME=localhost
export ROS_IP=127.0.0.1

echo "设置ROS环境变量:"
echo "  ROS_HOSTNAME: $ROS_HOSTNAME"
echo "  ROS_IP: $ROS_IP"
echo "  ROS_MASTER_URI: $ROS_MASTER_URI"

echo ""
echo "现在可以运行测试脚本了:"
echo "  python3 test_callback_simple.py"
echo ""
echo "或者重新启动后端服务:"
echo "  ./start_backend.sh"
