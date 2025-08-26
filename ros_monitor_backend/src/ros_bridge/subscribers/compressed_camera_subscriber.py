#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
压缩图像相机数据订阅器
专门处理CompressedImage格式的图像数据
"""

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
import base64
from typing import Callable, Dict, Any
import time
import logging

logger = logging.getLogger(__name__)

class CompressedCameraSubscriber:
    """压缩图像相机数据订阅器"""
    
    def __init__(self, topic: str, callback: Callable[[Dict[str, Any]], None]):
        self.topic = topic
        self.callback = callback
        self.subscriber = None
        self.frame_count = 0
        self.last_frame_time = 0
        self.is_active = False
        
        # 从topic提取相机ID，映射到前端期望的ID
        if 'left_camera' in topic:
            self.camera_id = 'left_camera'
        elif 'right_camera' in topic:
            self.camera_id = 'right_camera'
        else:
            # 从话题路径中提取相机ID
            topic_parts = topic.split('/')
            if len(topic_parts) >= 2:
                self.camera_id = topic_parts[-2]  # 例如: /left_camera/image/compressed -> left_camera
            else:
                self.camera_id = 'unknown_camera'
        
        # 图像压缩参数
        self.jpeg_quality = 70  # 降低JPEG质量以减少文件大小
        self.max_width = 640   # 降低预览宽度以减少数据量
        
        logger.info(f"创建压缩图像订阅器: topic={topic}, camera_id={self.camera_id}")
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
        """压缩图像消息回调函数 - 简化版本"""
        try:
            current_time = time.time()
            
            # 详细的调试日志
            logger.info(f"[{self.topic}] 收到压缩图像: format={msg.format}, data_size={len(msg.data)}")
            
            # 处理压缩图像 - 简化版本
            cv_image = self._process_compressed_image_simple(msg)
            
            if cv_image is not None:
                # 每帧都保存图像用于调试
                # self._save_debug_image(cv_image, self.frame_count)
                
                # 调整图像尺寸以减少数据量
                if cv_image.shape[1] > self.max_width:
                    scale = self.max_width / cv_image.shape[1]
                    new_height = int(cv_image.shape[0] * scale)
                    cv_image = cv2.resize(cv_image, (self.max_width, new_height))
                    logger.info(f"[{self.topic}] 图像尺寸调整为: {cv_image.shape}")
                
                # 构造真实的前端数据格式
                # 将OpenCV图像编码为JPEG格式
                _, jpeg_buffer = cv2.imencode('.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
                jpeg_data = base64.b64encode(jpeg_buffer).decode('utf-8')
                
                camera_data = {
                    'camera_id': self.camera_id,
                    'topic': self.topic,
                    'timestamp': msg.header.stamp.to_sec(),
                    'sequence': self.frame_count,
                    'encoding': 'jpeg',
                    'width': cv_image.shape[1],
                    'height': cv_image.shape[0],
                    'data': jpeg_data,  # 发送真实的JPEG图像数据
                    'compressed': True,
                    'compressed_size': len(jpeg_data),
                    'compression_ratio': len(jpeg_data) / (cv_image.shape[0] * cv_image.shape[1] * 3) * 100,
                    'frame_rate': 1.0 / (current_time - self.last_frame_time) if self.last_frame_time > 0 else 0.0
                }
                
                # 调用回调函数发送数据
                self.callback(camera_data)
                self.frame_count += 1
                self.last_frame_time = current_time
                
                logger.info(f"[{self.topic}] 图像处理成功，帧数: {self.frame_count}")
            else:
                logger.warning(f"[{self.topic}] 压缩图像处理失败")
            
        except Exception as e:
            logger.error(f"[{self.topic}] 压缩图像回调错误: {e}")
            import traceback
            logger.error(f"[{self.topic}] 错误详情: {traceback.format_exc()}")
    
    def _save_debug_image(self, cv_image, frame_count):
        """保存调试图像 - 简化版本"""
        try:
            import os
            # 创建调试图像目录
            debug_dir = "debug_images"
            if not os.path.exists(debug_dir):
                os.makedirs(debug_dir)
            
            # 保存图像 - 每帧都保存
            filename = f"{debug_dir}/{self.camera_id}_frame_{frame_count:04d}.jpg"
            success = cv2.imwrite(filename, cv_image)
            
            if success:
                logger.info(f"[{self.topic}] 调试图像已保存: {filename}, 大小: {cv_image.shape}")
            else:
                logger.warning(f"[{self.topic}] 保存调试图像失败: {filename}")
                
        except Exception as e:
            logger.warning(f"[{self.topic}] 保存调试图像失败: {e}")
            import traceback
            logger.warning(f"[{self.topic}] 保存失败详情: {traceback.format_exc()}")
    
    def _process_compressed_image_simple(self, msg: CompressedImage):
        """处理压缩图像数据 - 简化版本"""
        try:
            # 将压缩图像数据转换为numpy数组
            np_arr = np.frombuffer(msg.data, np.uint8)
            
            # 解码图像 - 简化版本
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if cv_image is not None:
                logger.info(f"[{self.topic}] 图像解码成功: {cv_image.shape}")
                return cv_image
            else:
                logger.error(f"[{self.topic}] 图像解码失败")
                return None
            
        except Exception as e:
            logger.error(f"[{self.topic}] 处理压缩图像时发生错误: {e}")
            return None
    
    def update_settings(self, preview_height: int = None, jpeg_quality: int = None):
        """更新相机设置"""
        if preview_height is not None:
            self.max_width = int(preview_height * 4/3)  # 假设4:3宽高比
            logger.info(f"[{self.topic}] 预览宽度更新为: {self.max_width}")
        
        if jpeg_quality is not None:
            self.jpeg_quality = max(1, min(100, jpeg_quality))
            logger.info(f"[{self.topic}] JPEG质量更新为: {self.jpeg_quality}")
    
    def get_status(self):
        """获取订阅器状态"""
        return {
            'topic': self.topic,
            'camera_id': self.camera_id,
            'is_active': self.is_active,
            'frame_count': self.frame_count,
            'last_frame_time': self.last_frame_time,
            'jpeg_quality': self.jpeg_quality,
            'max_width': self.max_width
        }
    
    def shutdown(self):
        """关闭订阅器"""
        try:
            if self.subscriber:
                self.subscriber.unregister()
                self.subscriber = None
            self.is_active = False
            logger.info(f"压缩图像相机订阅器关闭: {self.topic}")
        except Exception as e:
            logger.error(f"关闭压缩图像相机订阅器时发生错误 {self.topic}: {e}")