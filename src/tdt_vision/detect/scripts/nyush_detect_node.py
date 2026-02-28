#!/usr/bin/env python3
"""
NYUSH Robotics 检测节点 - 使用 NYUSH 训练的模型进行机器人和装甲板检测
发布 detect_result 话题，与 T-DT 系统兼容
"""

import sys
import os
from pathlib import Path

# 添加 NYUSH 项目路径（只添加代码路径，不添加 venv）
NYUSH_PATH = "/home/nyu/Desktop/NYUSH_Robotics_RM_RadarStation"
sys.path.insert(0, NYUSH_PATH)

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from vision_interface.msg import DetectResult
from cv_bridge import CvBridge
import cv2
import numpy as np

from detect_function import YOLOv5Detector


class NYUSHDetectNode(Node):
    def __init__(self):
        super().__init__('nyush_detect_node')
        
        self.get_logger().info("Initializing NYUSH Detect Node...")
        
        # 切换到 NYUSH 目录（模型路径是相对路径）
        os.chdir(NYUSH_PATH)
        
        # 加载模型
        self.get_logger().info("Loading car detection model...")
        self.car_detector = YOLOv5Detector(
            'models/car.engine',
            data='yaml/car.yaml',
            conf_thres=0.3,
            iou_thres=0.5,
            max_det=14,
            ui=False
        )
        
        self.get_logger().info("Loading armor detection model...")
        self.armor_detector = YOLOv5Detector(
            'models/armor.engine',
            data='yaml/armor.yaml',
            conf_thres=0.5,
            iou_thres=0.2,
            max_det=1,
            ui=False
        )
        
        self.bridge = CvBridge()
        
        # QoS 配置
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # 订阅相机图像
        self.image_sub = self.create_subscription(
            Image,
            'camera_image',
            self.image_callback,
            sensor_qos
        )
        
        # 发布检测结果
        self.result_pub = self.create_publisher(
            DetectResult,
            'detect_result',
            sensor_qos
        )
        
        # 发布标注图像
        self.image_pub = self.create_publisher(
            Image,
            'detect_image',
            sensor_qos
        )
        
        self.get_logger().info("NYUSH Detect Node initialized!")
    
    def image_callback(self, msg):
        try:
            # 转换 ROS 图像为 OpenCV
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 第一阶段：检测机器人
            car_detections = self.car_detector.predict(img)
            
            # 初始化检测结果
            detect_result = DetectResult()
            detect_result.header.stamp = msg.header.stamp
            
            # 初始化为 0（长度 6，对应 1-6 号机器人）
            detect_result.blue_x = [0.0] * 6
            detect_result.blue_y = [0.0] * 6
            detect_result.red_x = [0.0] * 6
            detect_result.red_y = [0.0] * 6
            
            if not car_detections:
                self.get_logger().info("No car detected")
            else:
                # 打印检测到的所有目标（包含所有原始检测）
                all_labels = [(d[0], round(d[2], 2)) for d in car_detections]
                self.get_logger().info(f"=== Raw detections: {all_labels} ===")
                
                # 对每个检测到的车辆进行装甲板检测
                car_count = 0
                for det in car_detections:
                    cls, (x, y, w, h), conf = det
                    if cls != 'car':
                        continue
                    
                    car_count += 1
                    
                    # 裁剪车辆区域
                    x1 = max(0, x)
                    y1 = max(0, y)
                    x2 = min(img.shape[1], x + w)
                    y2 = min(img.shape[0], y + h)
                    
                    self.get_logger().info(f"  Car {car_count}: conf={conf:.2f}, bbox=[{x1},{y1},{x2},{y2}]")
                    
                    # 先绘制车辆框（绿色）
                    cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 3)
                    cv2.putText(img, f"car:{conf:.2f}", (x1, y1 - 45),
                                cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 3)
                    
                    car_img = img[y1:y2, x1:x2]
                    if car_img.size == 0:
                        continue
                    
                    # 第二阶段：检测装甲板
                    armor_detections = self.armor_detector.predict(car_img)
                    
                    if armor_detections:
                        for armor_det in armor_detections:
                            armor_cls, (ax, ay, aw, ah), armor_conf = armor_det
                            
                            # 计算装甲板在原图中的中心位置
                            center_x = x1 + ax + aw / 2
                            center_y = y1 + ay + ah / 2
                            
                            self.get_logger().info(f"    -> Armor: {armor_cls}, conf={armor_conf:.2f}, center=({center_x:.0f},{center_y:.0f})")
                            
                            # 绘制装甲板标签
                            label_color = (255, 0, 0) if armor_cls.startswith('B') else (0, 0, 255)
                            cv2.putText(img, f"{armor_cls}:{armor_conf:.2f}", (x1, y1 - 10),
                                        cv2.FONT_HERSHEY_SIMPLEX, 1.2, label_color, 3)
                            
                            # 解析装甲板类别 (B1, B2, R1, R2, etc.)
                            if armor_cls.startswith('B'):
                                try:
                                    num = int(armor_cls[1])
                                    if 1 <= num <= 6:
                                        detect_result.blue_x[num - 1] = float(center_x)
                                        detect_result.blue_y[num - 1] = float(center_y)
                                    elif num == 7:  # 哨兵映射到位置6
                                        detect_result.blue_x[5] = float(center_x)
                                        detect_result.blue_y[5] = float(center_y)
                                except ValueError:
                                    pass
                            elif armor_cls.startswith('R'):
                                try:
                                    num = int(armor_cls[1])
                                    if 1 <= num <= 6:
                                        detect_result.red_x[num - 1] = float(center_x)
                                        detect_result.red_y[num - 1] = float(center_y)
                                    elif num == 7:  # 哨兵映射到位置6
                                        detect_result.red_x[5] = float(center_x)
                                        detect_result.red_y[5] = float(center_y)
                                except ValueError:
                                    pass
                    else:
                        self.get_logger().info(f"    -> No armor detected")
            
            # 发布检测结果
            self.result_pub.publish(detect_result)
            
            # 发布标注图像
            img_msg = self.bridge.cv2_to_imgmsg(img, "bgr8")
            img_msg.header = msg.header
            self.image_pub.publish(img_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error in detection: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = NYUSHDetectNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
