#!/usr/bin/env python3
"""
自动对齐 RViz 显示：订阅地图点云，RANSAC 拟合地面平面，发布 rm_frame_display -> rm_frame 校正 TF。
使点云在 RViz 中自动显示为水平（地面朝下）。
"""
import struct
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster


def pointcloud2_to_xyz(msg: PointCloud2) -> np.ndarray:
    """从 PointCloud2 提取 x,y,z，兼容常见格式。"""
    point_step = msg.point_step
    row_step = msg.row_step
    data = msg.data
    off_x = off_y = off_z = None
    for i, f in enumerate(msg.fields):
        if f.name == "x":
            off_x = f.offset
        elif f.name == "y":
            off_y = f.offset
        elif f.name == "z":
            off_z = f.offset
    if off_x is None or off_y is None or off_z is None:
        return np.zeros((0, 3))
    pts = []
    for i in range(0, len(data), point_step):
        if i + point_step > len(data):
            break
        x = struct.unpack_from("f", data, i + off_x)[0]
        y = struct.unpack_from("f", data, i + off_y)[0]
        z = struct.unpack_from("f", data, i + off_z)[0]
        if not (np.isnan(x) or np.isnan(y) or np.isnan(z)):
            pts.append([x, y, z])
    return np.array(pts, dtype=np.float64)


def fit_ground_plane_ransac(points: np.ndarray, n_iterations: int = 200, inlier_threshold: float = 0.05) -> np.ndarray:
    """RANSAC 拟合最大平面（通常为地面），返回平面法向量 (nx, ny, nz)，指向地面下方。"""
    num_points = len(points)
    if num_points < 10:
        return np.array([0.0, 0.0, 1.0])
    best_normal = np.array([0.0, 0.0, 1.0])
    best_inliers = 0
    for _ in range(n_iterations):
        idx = np.random.choice(num_points, 3, replace=False).astype(np.int64)
        p1, p2, p3 = points[idx]
        v1 = p2 - p1
        v2 = p3 - p1
        normal = np.cross(v1, v2)
        nnorm = np.linalg.norm(normal)
        if nnorm < 1e-8:
            continue
        normal = normal / nnorm
        d = -np.dot(normal, p1)
        dists = np.abs(points @ normal + d)
        inliers = np.sum(dists < inlier_threshold)
        if inliers > best_inliers:
            best_inliers = inliers
            best_normal = normal.copy()
    if best_inliers < 10:
        return np.array([0.0, 0.0, 1.0])
    return best_normal


def rotation_from_z_to_normal(normal: np.ndarray) -> np.ndarray:
    """计算从 (0,0,1) 到 normal 的旋转矩阵。使地面法向量对齐到 Z。"""
    z = np.array([0.0, 0.0, 1.0])
    n = normal / (np.linalg.norm(normal) + 1e-9)
    if np.dot(n, z) < 0:
        n = -n
    cos_a = np.clip(np.dot(z, n), -1.0, 1.0)
    if cos_a > 0.9999:
        return np.eye(3)
    axis = np.cross(z, n)
    axis_norm = np.linalg.norm(axis)
    if axis_norm < 1e-9:
        return np.eye(3)
    axis = axis / axis_norm
    angle = np.arccos(cos_a)
    K = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    R = np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * (K @ K)
    return R


def rotation_matrix_to_quaternion(R: np.ndarray) -> tuple:
    """3x3 旋转矩阵转四元数 (x, y, z, w)。"""
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    n = np.sqrt(x*x + y*y + z*z + w*w)
    return (x/n, y/n, z/n, w/n)


class DisplayAlignerNode(Node):
    def __init__(self):
        super().__init__("display_aligner_node")
        self.declare_parameter("map_topic", "/livox/map")
        self.declare_parameter("max_points", 50000)
        self.declare_parameter("z_low_percentile", 30.0)
        self.declare_parameter("inlier_threshold", 0.05)
        self.map_topic = self.get_parameter("map_topic").value
        self.max_points = self.get_parameter("max_points").value
        self.z_low_percentile = self.get_parameter("z_low_percentile").value
        self.inlier_threshold = self.get_parameter("inlier_threshold").value
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.aligned = False
        self._publish_identity()
        self.sub = self.create_subscription(
            PointCloud2, self.map_topic, self.callback, 10
        )
        self.get_logger().info(f"Display aligner: waiting for map on {self.map_topic}")

    def _publish_identity(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "rm_frame_display"
        t.child_frame_id = "rm_frame"
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

    def callback(self, msg: PointCloud2):
        if self.aligned:
            return
        try:
            pts = pointcloud2_to_xyz(msg)
        except Exception as e:
            self.get_logger().warn(f"Failed to read point cloud: {e}")
            return
        if len(pts) < 100:
            return
        if len(pts) > self.max_points:
            idx = np.random.choice(len(pts), self.max_points, replace=False)
            pts = pts[idx]
        z_thresh = np.percentile(pts[:, 2], self.z_low_percentile)
        ground_pts = pts[pts[:, 2] <= z_thresh]
        if len(ground_pts) < 50:
            ground_pts = pts
        normal = fit_ground_plane_ransac(ground_pts, inlier_threshold=self.inlier_threshold)
        if normal[2] < 0:
            normal = -normal
        R = rotation_from_z_to_normal(normal)
        qx, qy, qz, qw = rotation_matrix_to_quaternion(R.T)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "rm_frame_display"
        t.child_frame_id = "rm_frame"
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = float(qx)
        t.transform.rotation.y = float(qy)
        t.transform.rotation.z = float(qz)
        t.transform.rotation.w = float(qw)
        self.tf_broadcaster.sendTransform(t)
        self.aligned = True
        self.get_logger().info(
            f"Display aligner: auto-aligned (normal≈[{normal[0]:.3f},{normal[1]:.3f},{normal[2]:.3f}])"
        )


def main(args=None):
    rclpy.init(args=args)
    node = DisplayAlignerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
