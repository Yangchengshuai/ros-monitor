#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
压缩图像相机调试测试脚本
专门用于测试CompressedImage格式的数据接收和处理
"""

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import time
import logging
import os

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

class CompressedCameraDebugTester:
    """压缩图像相机调试测试器"""
    
    def __init__(self, topic: str):
        self.topic = topic
        self.bridge = CvBridge()
        self.subscriber = None
        self.frame_count = 0
        self.last_frame_time = 0
        self.is_active = False
        
        # 从topic提取相机ID
        if 'left_camera' in topic:
            self.camera_id = 'left_camera'
        elif 'right_camera' in topic:
            self.camera_id = 'right_camera'
        else:
            topic_parts = topic.split('/')
            if len(topic_parts) >= 2:
                self.camera_id = topic_parts[-2]
            else:
                self.camera_id = 'unknown_camera'
        
        # 创建调试图像目录
        self.debug_dir = f"debug_images_{self.camera_id}"
        if not os.path.exists(self.debug_dir):
            os.makedirs(self.debug_dir)
        
        self._setup_subscriber()
        
    def _setup_subscriber(self):
        """设置ROS订阅器"""
        try:
            self.subscriber = rospy.Subscriber(
                self.topic,
                CompressedImage,
                self._image_callback,
                queue_size=1
            )
            self.is_active = True
            logger.info(f"压缩图像订阅器创建成功: {self.topic} -> {self.camera_id}")
        except Exception as e:
            logger.error(f"创建压缩图像订阅器失败 {self.topic}: {e}")
            self.is_active = False
            
    def _image_callback(self, msg: CompressedImage):
        """压缩图像消息回调函数"""
        try:
            current_time = time.time()
            
            # 详细的调试日志
            logger.info(f"[{self.topic}] 收到压缩图像: format={msg.format}, data_size={len(msg.data)}, "
                       f"timestamp={msg.header.stamp.to_sec()}")
            
            # 限制帧率，避免过度处理
            if self.last_frame_time > 0 and current_time - self.last_frame_time < 0.5:  # 最大2fps
                logger.debug(f"[{self.topic}] 跳过帧，帧率限制")
                return
                
            self.last_frame_time = current_time
            
            # 处理压缩图像
            cv_image = self._process_compressed_image(msg)
            
            if cv_image is not None:
                # 保存调试图像
                self._save_debug_image(cv_image, self.frame_count)
                
                # 显示图像（如果可能）
                try:
                    cv2.imshow(f"Compressed Camera {self.camera_id}", cv_image)
                    cv2.waitKey(1)  # 非阻塞显示
                except Exception as e:
                    logger.warning(f"[{self.topic}] 显示图像失败: {e}")
                
                self.frame_count += 1
                logger.info(f"[{self.topic}] 图像处理成功，帧数: {self.frame_count}")
            else:
                logger.warning(f"[{self.topic}] 图像处理失败")
            
        except Exception as e:
            logger.error(f"[{self.topic}] 压缩图像回调错误: {e}")
    
    def _process_compressed_image(self, msg: CompressedImage):
        """处理压缩图像数据"""
        try:
            # 将压缩图像数据转换为numpy数组
            np_arr = np.frombuffer(msg.data, np.uint8)
            
            # 解码图像
            if msg.format == 'jpeg':
                cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                if cv_image is not None:
                    logger.info(f"[{self.topic}] JPEG解码成功，尺寸: {cv_image.shape}")
                    return cv_image
            elif msg.format == 'png':
                cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                if cv_image is not None:
                    logger.info(f"[{self.topic}] PNG解码成功，尺寸: {cv_image.shape}")
                    return cv_image
            else:
                # 尝试通用解码
                cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                if cv_image is not None:
                    logger.info(f"[{self.topic}] 通用解码成功，格式: {msg.format}, 尺寸: {cv_image.shape}")
                    return cv_image
            
            logger.error(f"[{self.topic}] 压缩图像解码失败，格式: {msg.format}")
            return None
            
        except Exception as e:
            logger.error(f"[{self.topic}] 处理压缩图像时发生错误: {e}")
            return None
    
    def _save_debug_image(self, cv_image, frame_count):
        """保存调试图像"""
        try:
            # 保存原始尺寸图像
            filename = f"{self.debug_dir}/frame_{frame_count:04d}.jpg"
            cv2.imwrite(filename, cv_image)
            logger.info(f"[{self.topic}] 调试图像已保存: {filename}")
            
            # 保存缩略图（如果图像太大）
            height, width = cv_image.shape[:2]
            if width > 800:
                scale = 800 / width
                new_width = int(width * scale)
                new_height = int(height * scale)
                thumbnail = cv2.resize(cv_image, (new_width, new_height))
                thumb_filename = f"{self.debug_dir}/thumb_frame_{frame_count:04d}.jpg"
                cv2.imwrite(thumb_filename, thumbnail)
                logger.info(f"[{self.topic}] 缩略图已保存: {thumb_filename}")
                
        except Exception as e:
            logger.warning(f"[{self.topic}] 保存调试图像失败: {e}")
    
    def get_status(self):
        """获取测试器状态"""
        return {
            'topic': self.topic,
            'camera_id': self.camera_id,
            'is_active': self.is_active,
            'frame_count': self.frame_count,
            'last_frame_time': self.last_frame_time,
            'debug_dir': self.debug_dir
        }
    
    def shutdown(self):
        """关闭测试器"""
        try:
            if self.subscriber:
                self.subscriber.unregister()
                self.subscriber = None
            self.is_active = False
            cv2.destroyAllWindows()
            logger.info(f"压缩图像测试器关闭: {self.topic}")
        except Exception as e:
            logger.error(f"关闭压缩图像测试器时发生错误 {self.topic}: {e}")

def main():
    """主函数"""
    logger.info("启动压缩图像相机调试测试器...")
    
    # 初始化ROS节点
    rospy.init_node('compressed_camera_debug_tester', anonymous=True)
    
    # 创建相机测试器
    camera_topics = [
        '/left_camera/image/compressed',
        '/right_camera/image/compressed'
    ]
    
    testers = []
    for topic in camera_topics:
        tester = CompressedCameraDebugTester(topic)
        testers.append(tester)
    
    logger.info("压缩图像相机调试测试器启动完成，等待图像数据...")
    logger.info("按 Ctrl+C 退出")
    
    try:
        # 主循环
        rate = rospy.Rate(1)  # 1Hz
        while not rospy.is_shutdown():
            # 打印状态
            for tester in testers:
                status = tester.get_status()
                if status['frame_count'] > 0:
                    logger.info(f"相机 {status['camera_id']}: 已接收 {status['frame_count']} 帧")
            
            rate.sleep()
            
    except KeyboardInterrupt:
        logger.info("收到退出信号")
    except Exception as e:
        logger.error(f"主循环发生错误: {e}")
    finally:
        # 清理资源
        for tester in testers:
            tester.shutdown()
        logger.info("压缩图像相机调试测试器已关闭")

if __name__ == '__main__':
    main()
