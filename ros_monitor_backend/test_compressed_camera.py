#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
压缩图像相机测试脚本
用于验证压缩图像数据接收和图像保存功能
"""

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
import time
import logging
import os

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

class CompressedCameraTester:
    """压缩图像相机测试器"""
    
    def __init__(self, topic: str, save_dir: str = "compressed_test_images"):
        self.topic = topic
        self.save_dir = save_dir
        self.subscriber = None
        self.frame_count = 0
        self.last_frame_time = 0
        self.is_active = False
        
        # 从topic提取相机ID
        self.camera_id = topic.split('/')[-3] if len(topic.split('/')) > 2 else 'unknown'
        
        # 创建保存目录
        os.makedirs(self.save_dir, exist_ok=True)
        
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
            logger.info(f"压缩图像相机订阅器创建成功: {self.topic}")
        except Exception as e:
            logger.error(f"创建压缩图像相机订阅器失败 {self.topic}: {e}")
            self.is_active = False
            
    def _image_callback(self, msg: CompressedImage):
        """压缩图像消息回调函数"""
        try:
            current_time = time.time()
            
            # 详细的调试日志
            logger.info(f"[{self.topic}] 收到压缩图像: format={msg.format}, data_size={len(msg.data)}")
            
            # 限制帧率，避免过度处理
            if self.last_frame_time > 0 and current_time - self.last_frame_time < 1.0:  # 最大1fps
                logger.debug(f"[{self.topic}] 跳过帧，帧率限制")
                return
                
            self.last_frame_time = current_time
            
            # 处理压缩图像
            cv_image = self._process_compressed_image(msg)
            
            if cv_image is not None:
                # 保存原始图像
                raw_filename = os.path.join(self.save_dir, f"raw_{self.camera_id}_{self.frame_count:04d}.jpg")
                cv2.imwrite(raw_filename, cv_image)
                logger.info(f"[{self.topic}] 原始图像保存成功: {raw_filename}")
                
                # 保存编码信息
                info_filename = os.path.join(self.save_dir, f"info_{self.camera_id}_{self.frame_count:04d}.txt")
                with open(info_filename, 'w') as f:
                    f.write(f"Topic: {self.topic}\n")
                    f.write(f"Format: {msg.format}\n")
                    f.write(f"Width: {cv_image.shape[1]}\n")
                    f.write(f"Height: {cv_image.shape[0]}\n")
                    f.write(f"Data Size: {len(msg.data)}\n")
                    f.write(f"Timestamp: {msg.header.stamp.to_sec()}\n")
                    f.write(f"Frame Count: {self.frame_count}\n")
                    f.write(f"Processing Time: {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
                
                # 显示图像
                try:
                    cv2.imshow(f"Camera {self.camera_id}", cv_image)
                    cv2.waitKey(1)
                except Exception as e:
                    logger.warning(f"[{self.topic}] 显示图像失败: {e}")
                
                self.frame_count += 1
                logger.info(f"[{self.topic}] 压缩图像处理成功，总帧数: {self.frame_count}")
            else:
                logger.warning(f"[{self.topic}] 压缩图像处理失败")
            
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
                    logger.info(f"[{self.topic}] JPEG解码成功")
                    return cv_image
            elif msg.format == 'png':
                cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                if cv_image is not None:
                    logger.info(f"[{self.topic}] PNG解码成功")
                    return cv_image
            else:
                # 尝试通用解码
                cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                if cv_image is not None:
                    logger.info(f"[{self.topic}] 通用解码成功，格式: {msg.format}")
                    return cv_image
            
            logger.error(f"[{self.topic}] 压缩图像解码失败，格式: {msg.format}")
            return None
            
        except Exception as e:
            logger.error(f"[{self.topic}] 处理压缩图像时发生错误: {e}")
            return None
    
    def get_status(self):
        """获取测试器状态"""
        return {
            'topic': self.topic,
            'camera_id': self.camera_id,
            'is_active': self.is_active,
            'frame_count': self.frame_count,
            'last_frame_time': self.last_frame_time,
            'save_dir': self.save_dir
        }
    
    def shutdown(self):
        """关闭测试器"""
        try:
            if self.subscriber:
                self.subscriber.unregister()
                self.subscriber = None
            self.is_active = False
            cv2.destroyAllWindows()
            logger.info(f"压缩图像相机测试器关闭: {self.topic}")
        except Exception as e:
            logger.error(f"关闭压缩图像相机测试器时发生错误 {self.topic}: {e}")

def main():
    """主函数"""
    logger.info("启动压缩图像相机测试器...")
    
    # 初始化ROS节点
    rospy.init_node('compressed_camera_tester', anonymous=True)
    
    # 创建相机测试器
    camera_topics = [
        '/left_camera/image/compressed'
    ]
    
    testers = []
    for topic in camera_topics:
        tester = CompressedCameraTester(topic)
        testers.append(tester)
    
    logger.info("压缩图像相机测试器启动完成，等待图像数据...")
    logger.info("按 Ctrl+C 退出")
    
    try:
        # 主循环
        rate = rospy.Rate(1)  # 1Hz
        while not rospy.is_shutdown():
            # 打印状态
            for tester in testers:
                status = tester.get_status()
                if status['frame_count'] > 0:
                    logger.info(f"相机 {status['camera_id']}: 已接收 {status['frame_count']} 帧，保存目录: {status['save_dir']}")
                else:
                    logger.info(f"相机 {status['camera_id']}: 等待数据...")
            
            rate.sleep()
            
    except KeyboardInterrupt:
        logger.info("收到退出信号")
    except Exception as e:
        logger.error(f"主循环发生错误: {e}")
    finally:
        # 清理资源
        for tester in testers:
            tester.shutdown()
        logger.info("压缩图像相机测试器已关闭")

if __name__ == '__main__':
    main()