#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
相机系统完整调试脚本
测试从ROS话题到前端显示的整个数据流
"""

import rospy
import time
import logging
import json
import requests
from sensor_msgs.msg import CompressedImage

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

class CameraSystemDebugger:
    """相机系统调试器"""
    
    def __init__(self):
        self.subscribers = {}
        self.frame_counts = {}
        self.last_frame_times = {}
        self.backend_url = "http://localhost:8000"
        
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
    
    def check_backend_status(self):
        """检查后端状态"""
        try:
            response = requests.get(f"{self.backend_url}/api/v1/system/status", timeout=5)
            if response.status_code == 200:
                data = response.json()
                logger.info("后端状态检查成功")
                logger.info(f"ROS连接状态: {data['data']['ros_connection']['running']}")
                logger.info(f"订阅器数量: {data['data']['ros_connection']['subscriber_count']}")
                logger.info(f"数据话题: {data['data']['ros_connection']['data_topics']}")
                return data
            else:
                logger.error(f"后端状态检查失败: {response.status_code}")
                return None
        except Exception as e:
            logger.error(f"后端状态检查异常: {e}")
            return None
    
    def check_camera_data(self):
        """检查相机数据"""
        try:
            response = requests.get(f"{self.backend_url}/api/v1/camera/data", timeout=5)
            if response.status_code == 200:
                data = response.json()
                logger.info("相机数据检查成功")
                logger.info(f"相机数量: {len(data.get('data', {}))}")
                return data
            else:
                logger.error(f"相机数据检查失败: {response.status_code}")
                return None
        except Exception as e:
            logger.error(f"相机数据检查异常: {e}")
            return None
    
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
        """关闭调试器"""
        for topic, subscriber in self.subscribers.items():
            try:
                subscriber.unregister()
                logger.info(f"订阅器已关闭: {topic}")
            except Exception as e:
                logger.error(f"关闭订阅器失败 {topic}: {e}")

def main():
    """主函数"""
    logger.info("启动相机系统完整调试...")
    
    # 初始化ROS节点
    rospy.init_node('camera_system_debugger', anonymous=True)
    
    # 创建调试器
    debugger = CameraSystemDebugger()
    
    logger.info("调试器启动完成，开始系统检查...")
    
    try:
        # 等待一段时间让系统稳定
        time.sleep(3)
        
        # 检查后端状态
        logger.info("=== 检查后端状态 ===")
        backend_status = debugger.check_backend_status()
        
        # 检查相机数据
        logger.info("=== 检查相机数据 ===")
        camera_data = debugger.check_camera_data()
        
        # 主循环
        rate = rospy.Rate(1)  # 1Hz
        while not rospy.is_shutdown():
            # 打印状态
            status = debugger.get_status()
            logger.info("=== ROS话题状态 ===")
            for topic, info in status.items():
                if info['frame_count'] > 0:
                    logger.info(f"话题 {topic}: 已接收 {info['frame_count']} 帧")
                else:
                    logger.warning(f"话题 {topic}: 未接收数据")
            
            # 每30秒检查一次后端状态
            if int(time.time()) % 30 == 0:
                logger.info("=== 定期检查后端状态 ===")
                debugger.check_backend_status()
            
            rate.sleep()
            
    except KeyboardInterrupt:
        logger.info("收到退出信号")
    except Exception as e:
        logger.error(f"主循环发生错误: {e}")
    finally:
        # 清理资源
        debugger.shutdown()
        logger.info("调试器已关闭")

if __name__ == '__main__':
    main()
