#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
后端相机数据接收测试脚本
验证ROS话题订阅和数据处理是否正常
"""

import rospy
import time
import logging
from sensor_msgs.msg import CompressedImage

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

class BackendCameraTester:
    """后端相机测试器"""
    
    def __init__(self):
        self.subscribers = {}
        self.frame_counts = {}
        self.last_frame_times = {}
        
        # 测试话题
        self.test_topics = [
            '/left_camera/image/compressed',
            '/right_camera/image/compressed'
        ]
        
        self._setup_subscribers()
        
    def _setup_subscribers(self):
        """设置订阅器"""
        for topic in self.test_topics:
            try:
                subscriber = rospy.Subscriber(
                    topic,
                    CompressedImage,
                    lambda msg, t=topic: self._image_callback(msg, t),
                    queue_size=1
                )
                self.subscribers[topic] = subscriber
                self.frame_counts[topic] = 0
                self.last_frame_times[topic] = 0
                logger.info(f"订阅器创建成功: {topic}")
            except Exception as e:
                logger.error(f"创建订阅器失败 {topic}: {e}")
    
    def _image_callback(self, msg: CompressedImage, topic: str):
        """图像回调函数"""
        try:
            current_time = time.time()
            
            # 限制帧率
            if self.last_frame_times[topic] > 0 and current_time - self.last_frame_times[topic] < 1.0:
                return
                
            self.last_frame_times[topic] = current_time
            self.frame_counts[topic] += 1
            
            logger.info(f"[{topic}] 收到图像: format={msg.format}, size={len(msg.data)}, "
                       f"timestamp={msg.header.stamp.to_sec()}, frame={self.frame_counts[topic]}")
            
        except Exception as e:
            logger.error(f"[{topic}] 回调错误: {e}")
    
    def get_status(self):
        """获取状态"""
        status = {}
        for topic in self.test_topics:
            status[topic] = {
                'frame_count': self.frame_counts.get(topic, 0),
                'last_frame_time': self.last_frame_times.get(topic, 0),
                'is_active': topic in self.subscribers
            }
        return status
    
    def shutdown(self):
        """关闭测试器"""
        for topic, subscriber in self.subscribers.items():
            try:
                subscriber.unregister()
                logger.info(f"订阅器已关闭: {topic}")
            except Exception as e:
                logger.error(f"关闭订阅器失败 {topic}: {e}")

def main():
    """主函数"""
    logger.info("启动后端相机数据接收测试...")
    
    # 初始化ROS节点
    rospy.init_node('backend_camera_tester', anonymous=True)
    
    # 创建测试器
    tester = BackendCameraTester()
    
    logger.info("测试器启动完成，等待图像数据...")
    logger.info("按 Ctrl+C 退出")
    
    try:
        # 主循环
        rate = rospy.Rate(1)  # 1Hz
        while not rospy.is_shutdown():
            # 打印状态
            status = tester.get_status()
            for topic, info in status.items():
                if info['frame_count'] > 0:
                    logger.info(f"话题 {topic}: 已接收 {info['frame_count']} 帧")
            
            rate.sleep()
            
    except KeyboardInterrupt:
        logger.info("收到退出信号")
    except Exception as e:
        logger.error(f"主循环发生错误: {e}")
    finally:
        # 清理资源
        tester.shutdown()
        logger.info("测试器已关闭")

if __name__ == '__main__':
    main()
