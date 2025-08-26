#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
简化的回调函数测试脚本
"""

import rospy
import time
import logging
from sensor_msgs.msg import CompressedImage

# 配置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def simple_callback(data):
    """简化的回调函数"""
    logger.info(f"=== 回调函数被调用 ===")
    logger.info(f"话题: {data._connection_header['topic']}")
    logger.info(f"格式: {data.format}")
    logger.info(f"数据大小: {len(data.data)}")
    logger.info(f"时间戳: {data.header.stamp}")
    logger.info(f"=====================")

def test_simple():
    """简单测试"""
    try:
        logger.info("开始简单测试...")
        
        # 初始化ROS节点
        rospy.init_node('test_simple', anonymous=True)
        logger.info("ROS节点初始化成功")
        
        # 创建订阅器
        topic = '/left_camera/image/compressed'
        subscriber = rospy.Subscriber(topic, CompressedImage, simple_callback, queue_size=1)
        logger.info(f"订阅器创建成功: {topic}")
        
        # 等待消息
        logger.info("等待图像消息...")
        rospy.spin()
        
    except Exception as e:
        logger.error(f"测试失败: {e}")
        import traceback
        traceback.print_exc()

if __name__ == '__main__':
    test_simple()
