#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
最小化ROS节点测试脚本
"""

import rospy
import logging

# 配置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def test_minimal():
    """最小化测试"""
    try:
        logger.info("开始最小化测试...")
        
        # 初始化ROS节点
        logger.info("尝试初始化ROS节点...")
        rospy.init_node('test_minimal', anonymous=True)
        logger.info("ROS节点初始化成功!")
        
        # 检查节点状态
        logger.info(f"节点名称: {rospy.get_name()}")
        logger.info(f"是否关闭: {rospy.is_shutdown()}")
        
        # 关闭节点
        rospy.signal_shutdown("测试完成")
        logger.info("ROS节点已关闭")
        
    except Exception as e:
        logger.error(f"测试失败: {e}")
        import traceback
        traceback.print_exc()

if __name__ == '__main__':
    test_minimal()
