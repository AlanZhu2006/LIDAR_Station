#!/usr/bin/env python3
import argparse
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, PointCloud2
from vision_interface.msg import DetectResult


QOS_BEST = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    durability=DurabilityPolicy.VOLATILE,
)

QOS_REL = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    durability=DurabilityPolicy.VOLATILE,
)


TOPICS = [
    ("/camera_image", "camera_image", Image, QOS_BEST),
    ("/detect_image", "detect_image", Image, QOS_BEST),
    ("/resolve_result", "resolve_result", DetectResult, QOS_BEST),
    ("/livox/lidar", "livox/lidar", PointCloud2, QOS_REL),
    ("/livox/lidar_dynamic", "livox/lidar_dynamic", PointCloud2, QOS_BEST),
    ("/livox/lidar_cluster", "livox/lidar_cluster", PointCloud2, QOS_BEST),
    ("/livox/lidar_kalman", "livox/lidar_kalman", PointCloud2, QOS_BEST),
]


class TopicProbe(Node):
    def __init__(self):
        super().__init__("latency_probe")
        self.samples = {
            topic_name: {"ages": [], "recv_times": []} for topic_name, _, _, _ in TOPICS
        }
        self.subscribers = []
        for topic_name, _, msg_type, qos in TOPICS:
            sub = self.create_subscription(
                msg_type, topic_name, self._make_callback(topic_name), qos
            )
            self.subscribers.append(sub)

    def _make_callback(self, topic_name):
        def callback(msg):
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            now = time.time()
            age_ms = (now - stamp) * 1000.0
            if -1000.0 < age_ms < 10000.0:
                self.samples[topic_name]["ages"].append(age_ms)
                self.samples[topic_name]["recv_times"].append(now)

        return callback


def percentile(values, p):
    if not values:
        return None
    ordered = sorted(values)
    idx = max(0, min(len(ordered) - 1, int(len(ordered) * p) - 1))
    return ordered[idx]


def summarize(label, ages, recv_times):
    if not ages:
        return f"   {label}: 无数据"
    if len(recv_times) >= 2 and recv_times[-1] > recv_times[0]:
        rate_hz = (len(recv_times) - 1) / (recv_times[-1] - recv_times[0])
    else:
        rate_hz = 0.0
    avg_age = sum(ages) / len(ages)
    p95_age = percentile(ages, 0.95)
    latest_age = ages[-1]
    return (
        f"   {label}: 频率 {rate_hz:.2f} Hz | 平均年龄 {avg_age:.1f} ms | "
        f"P95 {p95_age:.1f} ms | 最新 {latest_age:.1f} ms"
    )


def main():
    parser = argparse.ArgumentParser(description="QoS-aware latency probe for fusion topics")
    parser.add_argument("--duration", type=float, default=4.0, help="Sampling duration in seconds")
    args = parser.parse_args()

    rclpy.init()
    node = TopicProbe()
    end_time = time.time() + args.duration
    while time.time() < end_time:
        rclpy.spin_once(node, timeout_sec=0.1)

    print("1. 话题真实频率与消息年龄（QoS 兼容采样）")
    for topic_name, label, _, _ in TOPICS:
        data = node.samples[topic_name]
        print(summarize(label, data["ages"], data["recv_times"]))

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
