#!/usr/bin/env python3
"""
用 OpenCV 显示 /camera_image 或 /detect_image，不占用 RViz 资源。
用法: python3 scripts/view_camera_opencv.py [topic]
默认: /camera_image
"""
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CameraViewer(Node):
    def __init__(self, topic='/camera_image'):
        super().__init__('camera_viewer')
        self.bridge = CvBridge()
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        self.sub = self.create_subscription(
            Image, topic, self.callback, qos
        )
        self.get_logger().info(f'Viewing {topic} (press q to quit)')

    def callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.imshow('Camera', cv_image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                rclpy.shutdown()
        except Exception as e:
            self.get_logger().warn(f'Callback error: {e}')


def main():
    topic = sys.argv[1] if len(sys.argv) > 1 else '/camera_image'
    rclpy.init()
    node = CameraViewer(topic)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
