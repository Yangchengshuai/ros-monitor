import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import base64
from typing import Callable, Dict, Any
import json
import time
import logging

logger = logging.getLogger(__name__)

class CameraSubscriber:
    """相机数据订阅器 - 专门处理BayerGB12Packed等格式的图像数据"""
    
    def __init__(self, topic: str, callback: Callable[[Dict[str, Any]], None]):
        self.topic = topic
        self.callback = callback
        self.bridge = CvBridge()
        self.subscriber = None
        self.frame_count = 0
        self.last_frame_time = 0
        self.is_active = False
        
        # 从topic提取相机ID
        self.camera_id = topic.split('/')[-2] if len(topic.split('/')) > 1 else 'unknown'
        
        # 图像压缩参数
        self.jpeg_quality = 80
        self.max_width = 1920  # 最大预览宽度，支持高清图像
        #self.max_width = 640  # 最大预览宽度
        
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
            self.is_active = True
            logger.info(f"相机订阅器创建成功: {self.topic}")
        except Exception as e:
            logger.error(f"创建相机订阅器失败 {self.topic}: {e}")
            self.is_active = False
            
    def _image_callback(self, msg: Image):
        """图像消息回调函数 - 处理多种图像格式并转换为前端可用的格式"""
        try:
            current_time = time.time()
            
            # 限制帧率，避免过度处理
            if self.last_frame_time > 0 and current_time - self.last_frame_time < 0.1:  # 最大10fps
                return
                
            self.last_frame_time = current_time
            
            # 详细的调试日志
            logger.info(f"[{self.topic}] 收到图像: encoding={msg.encoding}, width={msg.width}, height={msg.height}, "
                       f"timestamp={msg.header.stamp.to_sec()}, data_size={len(msg.data)}")
            
            # 尝试处理图像
            cv_image = self._try_process_image(msg)
            
            if cv_image is not None:
                # 压缩和编码图像
                encoded_image, compressed_size = self._compress_and_encode_image(cv_image)
                
                if encoded_image:
                    # 构造前端期望的数据格式
                    camera_data = {
                        'camera_id': self.camera_id,
                        'topic': self.topic,
                        'timestamp': msg.header.stamp.to_sec(),
                        'sequence': self.frame_count,
                        'encoding': 'jpeg',
                        'width': cv_image.shape[1],
                        'height': cv_image.shape[0],
                        'data': encoded_image,
                        'compressed': False,
                        'compressed_size': compressed_size,
                        'compression_ratio': len(msg.data) / compressed_size if compressed_size > 0 else 1.0,
                        'frame_rate': 1.0 / (current_time - self.last_frame_time) if self.last_frame_time > 0 else 0.0
                    }
                    
                    # 调用回调函数发送数据
                    self.callback(camera_data)
                    self.frame_count += 1
                    
                    logger.info(f"[{self.topic}] 图像处理成功，帧数: {self.frame_count}")
                else:
                    logger.warning(f"[{self.topic}] 图像编码失败")
            else:
                logger.warning(f"[{self.topic}] 图像处理失败")
            
        except Exception as e:
            logger.error(f"[{self.topic}] 相机回调错误: {e}")
    
    def _try_process_image(self, msg: Image):
        """尝试处理图像，使用多种方法处理不同的编码格式"""
        try:
            # 针对BayerGB12Packed格式的特殊处理（根据配置文件，这是主要格式）
            if msg.encoding == 'bayer_gb12p':
                logger.debug(f"[{self.topic}] 处理BayerGB12Packed格式图像")
                
                # 方法1: 尝试直接转换为BGR8
                try:
                    cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                    if cv_image is not None:
                        logger.info(f"[{self.topic}] 方法1成功: 直接转换为BGR8")
                        return cv_image
                except Exception as e:
                    logger.debug(f"[{self.topic}] 方法1失败: {e}")
                
                # 方法2: 先转换为16位Bayer，再转换为BGR
                try:
                    cv_image_16 = self.bridge.imgmsg_to_cv2(msg, "bayer_gb16")
                    if cv_image_16 is not None:
                        cv_image = cv2.cvtColor(cv_image_16, cv2.COLOR_BayerGB2BGR)
                        # 如果图像是16位，转换为8位
                        if cv_image.dtype == np.uint16:
                            cv_image = (cv_image / 256).astype(np.uint8)
                        logger.info(f"[{self.topic}] 方法2成功: 16位Bayer转换")
                        return cv_image
                except Exception as e:
                    logger.debug(f"[{self.topic}] 方法2失败: {e}")
                
                # 方法3: 尝试转换为mono8，然后创建彩色图像
                try:
                    cv_image_mono = self.bridge.imgmsg_to_cv2(msg, "mono8")
                    if cv_image_mono is not None:
                        # 创建彩色图像（复制单通道到三个通道）
                        cv_image = cv2.cvtColor(cv_image_mono, cv2.COLOR_GRAY2BGR)
                        logger.info(f"[{self.topic}] 方法3成功: mono8转换")
                        return cv_image
                except Exception as e:
                    logger.debug(f"[{self.topic}] 方法3失败: {e}")
                
                # 方法4: 尝试转换为bayer_gb8
                try:
                    cv_image = self.bridge.imgmsg_to_cv2(msg, "bayer_gb8")
                    if cv_image is not None:
                        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BayerGB2BGR)
                        logger.info(f"[{self.topic}] 方法4成功: bayer_gb8转换")
                        return cv_image
                except Exception as e:
                    logger.debug(f"[{self.topic}] 方法4失败: {e}")
                
                # 方法5: 尝试转换为bayer_rg8
                try:
                    cv_image = self.bridge.imgmsg_to_cv2(msg, "bayer_rg8")
                    if cv_image is not None:
                        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BayerRG2BGR)
                        logger.info(f"[{self.topic}] 方法5成功: bayer_rg8转换")
                        return cv_image
                except Exception as e:
                    logger.debug(f"[{self.topic}] 方法5失败: {e}")
                
                logger.error(f"[{self.topic}] 所有BayerGB12Packed处理方法都失败")
                return None
            
            # 处理其他格式
            elif msg.encoding == 'bayer_gb8':
                logger.debug(f"[{self.topic}] 处理BayerGB8格式图像")
                try:
                    cv_image = self.bridge.imgmsg_to_cv2(msg, "bayer_gb8")
                    if cv_image is not None:
                        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BayerGB2BGR)
                        return cv_image
                except Exception as e:
                    logger.warning(f"[{self.topic}] BayerGB8转换失败: {e}")
            
            elif msg.encoding == 'bayer_rg8':
                logger.debug(f"[{self.topic}] 处理BayerRG8格式图像")
                try:
                    cv_image = self.bridge.imgmsg_to_cv2(msg, "bayer_rg8")
                    if cv_image is not None:
                        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BayerRG2BGR)
                        return cv_image
                except Exception as e:
                    logger.warning(f"[{self.topic}] BayerRG8转换失败: {e}")
            
            elif msg.encoding == 'rgb8':
                logger.debug(f"[{self.topic}] 处理RGB8格式图像")
                try:
                    cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
                    if cv_image is not None:
                        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
                        return cv_image
                except Exception as e:
                    logger.warning(f"[{self.topic}] RGB8转换失败: {e}")
            
            # 通用方法：尝试转换为BGR8
            try:
                logger.info(f"[{self.topic}] 尝试通用BGR8转换")
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                if cv_image is not None:
                    return cv_image
            except Exception as e:
                logger.warning(f"[{self.topic}] 通用BGR8转换失败: {e}")
            
            # 如果所有方法都失败，创建测试图像
            logger.warning(f"[{self.topic}] 所有转换方法失败，创建测试图像")
            return self._create_test_image(msg.width, msg.height)
            
        except Exception as e:
            logger.error(f"[{self.topic}] 图像处理过程中发生错误: {e}")
            return None
    
    def _compress_and_encode_image(self, cv_image):
        """压缩和编码图像为JPEG格式的Base64字符串"""
        try:
            # 调整图像大小以提高传输效率
            height, width = cv_image.shape[:2]
            if width > self.max_width:
                scale = self.max_width / width
                new_width = int(width * scale)
                new_height = int(height * scale)
                cv_image = cv2.resize(cv_image, (new_width, new_height))
                logger.info(f"[{self.topic}] 图像缩放: {width}x{height} -> {new_width}x{new_height}")
            
            # 编码为JPEG
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
            success, encoded_image = cv2.imencode('.jpg', cv_image, encode_param)
            
            if success:
                # 转换为Base64
                base64_string = base64.b64encode(encoded_image.tobytes()).decode('utf-8')
                compressed_size = len(encoded_image.tobytes())
                logger.info(f"[{self.topic}] 图像编码成功，压缩后大小: {compressed_size} bytes")
                return base64_string, compressed_size
            else:
                logger.error(f"[{self.topic}] 图像编码失败")
                return None, 0
                
        except Exception as e:
            logger.error(f"[{self.topic}] 图像压缩编码失败: {e}")
            return None, 0
    
    def _create_test_image(self, width: int, height: int):
        """创建测试图像，用于调试"""
        try:
            img = np.zeros((height, width, 3), dtype=np.uint8)
            
            # 添加一些测试内容
            cv2.putText(img, f'Test Image', (10, height//2 - 20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
            cv2.putText(img, f'{width}x{height}', (10, height//2 + 20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2)
            cv2.putText(img, f'Time: {time.strftime("%H:%M:%S")}', (10, height//2 + 50), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 150, 150), 1)
            cv2.putText(img, f'Topic: {self.topic}', (10, height//2 + 80), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 100, 100), 1)
            
            # 添加一些彩色区域
            cv2.rectangle(img, (width//4, height//4), (3*width//4, 3*height//4), (0, 255, 0), 2)
            cv2.circle(img, (width//2, height//2), min(width, height)//6, (255, 0, 0), -1)
            
            return img
        except Exception as e:
            logger.error(f"[{self.topic}] 创建测试图像失败: {e}")
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
            logger.info(f"相机订阅器关闭: {self.topic}")
        except Exception as e:
            logger.error(f"关闭相机订阅器时发生错误 {self.topic}: {e}")
