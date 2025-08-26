#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS节点初始化测试脚本
"""

import rospy
import time
import logging

# 配置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def test_ros_init():
    """测试ROS节点初始化"""
    try:
        logger.info("开始测试ROS节点初始化...")
        
        # 检查ROS环境
        logger.info(f"ROS_DISTRO: {rospy.get_param('/rosdistro', 'unknown')}")
        logger.info(f"ROS_MASTER_URI: {rospy.get_master().getUri()}")
        
        # 尝试初始化节点
        logger.info("尝试初始化ROS节点...")
        rospy.init_node('test_ros_init', anonymous=True, disable_signals=True)
        logger.info("ROS节点初始化成功!")
        
        # 检查节点状态
        logger.info(f"节点名称: {rospy.get_name()}")
        logger.info(f"节点命名空间: {rospy.get_namespace()}")
        logger.info(f"是否关闭: {rospy.is_shutdown()}")
        
        # 等待一段时间
        logger.info("等待5秒...")
        time.sleep(5)
        
        # 再次检查状态
        logger.info(f"5秒后是否关闭: {rospy.is_shutdown()}")
        
        # 关闭节点
        rospy.signal_shutdown("测试完成")
        logger.info("ROS节点已关闭")
        
    except Exception as e:
        logger.error(f"ROS节点初始化失败: {e}")
        import traceback
        traceback.print_exc()

if __name__ == '__main__':
    test_ros_init()
