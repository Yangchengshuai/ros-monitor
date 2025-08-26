#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
直接测试订阅器脚本
"""

import rospy
import time
import logging
from sensor_msgs.msg import CompressedImage

# 配置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def test_callback(data):
    """测试回调函数"""
    logger.info(f"收到图像数据: format={data.format}, size={len(data.data)}")

def test_subscriber():
    """测试订阅器"""
    try:
        logger.info("开始测试订阅器...")
        
        # 初始化ROS节点
        rospy.init_node('test_subscriber', anonymous=True)
        logger.info("ROS节点初始化成功")
        
        # 创建订阅器
        topic = '/left_camera/image/compressed'
        subscriber = rospy.Subscriber(topic, CompressedImage, test_callback, queue_size=1)
        logger.info(f"订阅器创建成功: {topic}")
        
        # 等待消息
        logger.info("等待图像消息...")
        rospy.spin()
        
    except Exception as e:
        logger.error(f"测试失败: {e}")
        import traceback
        traceback.print_exc()

if __name__ == '__main__':
    test_subscriber()
