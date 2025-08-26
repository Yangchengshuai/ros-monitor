#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
相机话题调试脚本
用于检查相机话题的状态和数据
"""

import rospy
import time
from sensor_msgs.msg import Image

def image_callback(msg, topic_name):
    """简单的图像回调函数"""
    print(f"[{topic_name}] 收到图像:")
    print(f"  编码: {msg.encoding}")
    print(f"  尺寸: {msg.width} x {msg.height}")
    print(f"  数据大小: {len(msg.data)} bytes")
    print(f"  时间戳: {msg.header.stamp.to_sec()}")
    print(f"  帧ID: {msg.header.seq}")
    print("-" * 50)

def main():
    """主函数"""
    print("启动相机话题调试器...")
    
    # 初始化ROS节点
    rospy.init_node('camera_topic_debugger', anonymous=True)
    
    # 相机话题列表
    camera_topics = [
        '/left_camera/image',
        '/right_camera/image'
    ]
    
    subscribers = []
    
    # 为每个话题创建订阅器
    for topic in camera_topics:
        try:
            # 使用lambda来传递topic_name参数
            sub = rospy.Subscriber(
                topic, 
                Image, 
                lambda msg, t=topic: image_callback(msg, t),
                queue_size=1
            )
            subscribers.append(sub)
            print(f"已订阅话题: {topic}")
        except Exception as e:
            print(f"订阅话题 {topic} 失败: {e}")
    
    if not subscribers:
        print("错误: 没有成功订阅任何话题")
        return
    
    print(f"成功订阅 {len(subscribers)} 个话题")
    print("等待图像数据...")
    print("按 Ctrl+C 退出")
    
    try:
        # 主循环
        rate = rospy.Rate(1)  # 1Hz
        while not rospy.is_shutdown():
            # 检查话题状态
            for topic in camera_topics:
                try:
                    # 获取话题信息
                    topic_info = rospy.get_published_topics()
                    topic_found = False
                    for t, msg_type in topic_info:
                        if t == topic:
                            topic_found = True
                            break
                    
                    if topic_found:
                        # 获取话题的发布者数量
                        publishers = rospy.get_publishers_by_type(topic, Image)
                        print(f"[{topic}] 状态: 活跃, 发布者数量: {len(publishers)}")
                    else:
                        print(f"[{topic}] 状态: 未找到")
                        
                except Exception as e:
                    print(f"[{topic}] 检查状态失败: {e}")
            
            rate.sleep()
            
    except KeyboardInterrupt:
        print("\n收到退出信号")
    except Exception as e:
        print(f"主循环发生错误: {e}")
    finally:
        # 清理资源
        for sub in subscribers:
            sub.unregister()
        print("调试器已关闭")

if __name__ == '__main__':
    main()

