#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
简化的相机调试脚本
用于快速验证相机数据接收
"""

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time
import logging

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

class SimpleCameraDebugger:
    """简化的相机调试器"""
    
    def __init__(self, topic: str):
        self.topic = topic
        self.bridge = CvBridge()
        self.subscriber = None
        self.frame_count = 0
        self.last_frame_time = 0
        self.is_active = False
        
        # 从topic提取相机ID
        self.camera_id = topic.split('/')[-2] if len(topic.split('/')) > 1 else 'unknown'
        
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
        """图像消息回调函数"""
        try:
            current_time = time.time()
            
            # 详细的调试日志
            logger.info(f"[{self.topic}] 收到图像: encoding={msg.encoding}, width={msg.width}, height={msg.height}, "
                       f"timestamp={msg.header.stamp.to_sec()}, data_size={len(msg.data)}")
            
            # 限制帧率，避免过度处理
            if self.last_frame_time > 0 and current_time - self.last_frame_time < 2.0:  # 最大0.5fps
                logger.debug(f"[{self.topic}] 跳过帧，帧率限制")
                return
                
            self.last_frame_time = current_time
            
            # 尝试处理图像
            cv_image = self._try_process_image(msg)
            
            if cv_image is not None:
                # 保存图像到文件
                filename = f"debug_image_{self.camera_id}_{self.frame_count:04d}.jpg"
                cv2.imwrite(filename, cv_image)
                logger.info(f"[{self.topic}] 图像保存成功: {filename}")
                
                # 显示图像
                try:
                    cv2.imshow(f"Camera {self.camera_id}", cv_image)
                    cv2.waitKey(1)
                except Exception as e:
                    logger.warning(f"[{self.topic}] 显示图像失败: {e}")
                
                self.frame_count += 1
                logger.info(f"[{self.topic}] 图像处理成功，总帧数: {self.frame_count}")
            else:
                logger.warning(f"[{self.topic}] 图像处理失败")
            
        except Exception as e:
            logger.error(f"[{self.topic}] 相机回调错误: {e}")
    
    def _try_process_image(self, msg: Image):
        """尝试处理图像，使用多种方法"""
        try:
            # 针对BayerGB12Packed格式的特殊处理
            if msg.encoding == 'bayer_gb12p':
                logger.info(f"[{self.topic}] 处理BayerGB12Packed格式图像")
                
                # 方法1: 尝试直接转换为BGR8
                try:
                    cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                    if cv_image is not None:
                        logger.info(f"[{self.topic}] 方法1成功: 直接转换为BGR8")
                        return cv_image
                except Exception as e:
                    logger.warning(f"[{self.topic}] 方法1失败: {e}")
                
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
                    logger.warning(f"[{self.topic}] 方法2失败: {e}")
                
                # 方法3: 尝试转换为mono8，然后创建彩色图像
                try:
                    cv_image_mono = self.bridge.imgmsg_to_cv2(msg, "mono8")
                    if cv_image_mono is not None:
                        # 创建彩色图像（复制单通道到三个通道）
                        cv_image = cv2.cvtColor(cv_image_mono, cv2.COLOR_GRAY2BGR)
                        logger.info(f"[{self.topic}] 方法3成功: mono8转换")
                        return cv_image
                except Exception as e:
                    logger.warning(f"[{self.topic}] 方法3失败: {e}")
                
                # 方法4: 尝试转换为bayer_gb8
                try:
                    cv_image = self.bridge.imgmsg_to_cv2(msg, "bayer_gb8")
                    if cv_image is not None:
                        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BayerGB2BGR)
                        logger.info(f"[{self.topic}] 方法4成功: bayer_gb8转换")
                        return cv_image
                except Exception as e:
                    logger.warning(f"[{self.topic}] 方法4失败: {e}")
                
                # 方法5: 尝试转换为bayer_rg8
                try:
                    cv_image = self.bridge.imgmsg_to_cv2(msg, "bayer_rg8")
                    if cv_image is not None:
                        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BayerRG2BGR)
                        logger.info(f"[{self.topic}] 方法5成功: bayer_rg8转换")
                        return cv_image
                except Exception as e:
                    logger.warning(f"[{self.topic}] 方法5失败: {e}")
                
                logger.error(f"[{self.topic}] 所有BayerGB12Packed处理方法都失败")
                return None
            
            # 处理其他格式
            elif msg.encoding == 'bayer_gb8':
                logger.info(f"[{self.topic}] 处理BayerGB8格式图像")
                try:
                    cv_image = self.bridge.imgmsg_to_cv2(msg, "bayer_gb8")
                    if cv_image is not None:
                        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BayerGB2BGR)
                        return cv_image
                except Exception as e:
                    logger.warning(f"[{self.topic}] BayerGB8转换失败: {e}")
            
            elif msg.encoding == 'bayer_rg8':
                logger.info(f"[{self.topic}] 处理BayerRG8格式图像")
                try:
                    cv_image = self.bridge.imgmsg_to_cv2(msg, "bayer_rg8")
                    if cv_image is not None:
                        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BayerRG2BGR)
                        return cv_image
                except Exception as e:
                    logger.warning(f"[{self.topic}] BayerRG8转换失败: {e}")
            
            elif msg.encoding == 'rgb8':
                logger.info(f"[{self.topic}] 处理RGB8格式图像")
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
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 150, 200), 1)
            cv2.putText(img, f'Topic: {self.topic}', (10, height//2 + 80), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 100, 100), 1)
            
            # 添加一些彩色区域
            cv2.rectangle(img, (width//4, height//4), (3*width//4, 3*height//4), (0, 255, 0), 2)
            cv2.circle(img, (width//2, height//2), min(width, height)//6, (255, 0, 0), -1)
            
            return img
        except Exception as e:
            logger.error(f"[{self.topic}] 创建测试图像失败: {e}")
            return None
    
    def get_status(self):
        """获取调试器状态"""
        return {
            'topic': self.topic,
            'camera_id': self.camera_id,
            'is_active': self.is_active,
            'frame_count': self.frame_count,
            'last_frame_time': self.last_frame_time
        }
    
    def shutdown(self):
        """关闭调试器"""
        try:
            if self.subscriber:
                self.subscriber.unregister()
                self.subscriber = None
            self.is_active = False
            cv2.destroyAllWindows()
            logger.info(f"相机调试器关闭: {self.topic}")
        except Exception as e:
            logger.error(f"关闭相机调试器时发生错误 {self.topic}: {e}")

def main():
    """主函数"""
    logger.info("启动简化的相机调试器...")
    
    # 初始化ROS节点
    rospy.init_node('simple_camera_debugger', anonymous=True)
    
    # 创建相机调试器
    camera_topics = [
        '/left_camera/image',
        '/right_camera/image'
    ]
    
    testers = []
    for topic in camera_topics:
        tester = SimpleCameraDebugger(topic)
        testers.append(tester)
    
    logger.info("相机调试器启动完成，等待图像数据...")
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
        logger.info("相机调试器已关闭")

if __name__ == '__main__':
    main()