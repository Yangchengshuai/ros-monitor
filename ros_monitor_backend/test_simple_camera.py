#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
import time
import logging

# 配置日志
logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)

class SimpleCameraTest:
    """简单相机测试"""
    
    def __init__(self, topic: str):
        self.topic = topic
        self.subscriber = None
        self.frame_count = 0
        self.last_frame_time = 0
        
        self._setup_subscriber()
        
    def _setup_subscriber(self):
        """设置ROS订阅器"""
        try:
            self.subscriber = rospy.Subscriber(
                self.topic,
                Image,
                self._image_callback,
                queue_size=1
            )
            logger.info(f"Simple camera subscriber created for {self.topic}")
        except Exception as e:
            logger.error(f"Failed to create subscriber for {self.topic}: {e}")
    
    def _image_callback(self, msg: Image):
        """图像消息回调函数"""
        try:
            current_time = time.time()
            self.frame_count += 1
            
            logger.info(f"=== RECEIVED IMAGE #{self.frame_count} ===")
            logger.info(f"Topic: {self.topic}")
            logger.info(f"Encoding: {msg.encoding}")
            logger.info(f"Width: {msg.width}")
            logger.info(f"Height: {msg.height}")
            logger.info(f"Data size: {len(msg.data)} bytes")
            logger.info(f"Timestamp: {msg.header.stamp.to_sec()}")
            logger.info(f"Current time: {current_time}")
            logger.info(f"Time since last: {current_time - self.last_frame_time if self.last_frame_time > 0 else 'N/A'}")
            logger.info("=" * 50)
            
            self.last_frame_time = current_time
            
        except Exception as e:
            logger.error(f"Simple callback error: {e}")

def main():
    """主函数"""
    rospy.init_node('simple_camera_test', anonymous=True)
    
    # 测试左相机
    left_camera = SimpleCameraTest('/left_camera/image')
    
    logger.info("Simple camera test started. Press Ctrl+C to stop.")
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        logger.info("Test stopped by user")
    except Exception as e:
        logger.error(f"Test error: {e}")

if __name__ == '__main__':
    main()

