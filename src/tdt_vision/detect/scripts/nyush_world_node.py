#!/usr/bin/python3.10
"""
NYUSH vision bridge for TDT fusion.

This node runs the NYUSH two-stage detector, applies the NYUSH map calibration
pipeline, and publishes world-coordinate detections directly on /resolve_result
using the TDT DetectResult message contract.
"""

import os
import sys
import time
import json
import importlib
from collections import deque
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import Image
from vision_interface.msg import DetectResult


DEFAULT_NYUSH_PATH = "/home/nyu/Desktop/NYUSH_Robotics_RM_RadarStation"
TESTMAP_FIELD_WIDTH_M = 6.79
TESTMAP_FIELD_HEIGHT_M = 3.82


class SlidingWindowFilter:
    def __init__(self, window_size: int, max_inactive_time: float):
        self.window_size = window_size
        self.max_inactive_time = max_inactive_time
        self.window = {}
        self.last_update = {}

    def add_data(self, name: str, x: float, y: float) -> None:
        if name not in self.window:
            self.window[name] = deque(maxlen=self.window_size)
        self.window[name].append((x, y))
        self.last_update[name] = time.time()

    def get_all_data(self):
        filtered = {}
        now = time.time()
        for name, samples in list(self.window.items()):
            if now - self.last_update.get(name, 0.0) > self.max_inactive_time:
                samples.clear()
                continue
            if len(samples) < self.window_size:
                filtered[name] = None
                continue
            avg_x = sum(point[0] for point in samples) / len(samples)
            avg_y = sum(point[1] for point in samples) / len(samples)
            filtered[name] = (avg_x, avg_y)
        return filtered


class NYUSHWorldNode(Node):
    def __init__(self):
        super().__init__("nyush_world_node")

        self.declare_parameter("nyush_path", os.environ.get("NYUSH_PATH", DEFAULT_NYUSH_PATH))
        self.declare_parameter("map_mode", "testmap")
        self.declare_parameter("state", "B")
        # `my_map(m).jpg` is the rotated runtime map for the NYUSH test field.
        # Its horizontal axis is the 6.79 m field width and its vertical axis
        # is the 3.82 m field height.
        self.declare_parameter("field_width_m", TESTMAP_FIELD_WIDTH_M)
        self.declare_parameter("field_height_m", TESTMAP_FIELD_HEIGHT_M)
        self.declare_parameter("calibration_width_px", 0)
        self.declare_parameter("calibration_height_px", 0)
        self.declare_parameter("window_size", 3)
        self.declare_parameter("max_inactive_time", 2.0)
        self.declare_parameter("diagnostic_log_every_sec", 2.0)
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("publish_debug_map", False)
        self.declare_parameter("debug_image_every_n", 1)
        self.declare_parameter("process_width", 0)
        self.declare_parameter("process_height", 0)
        self.declare_parameter("process_interval_ms", 5)
        self.declare_parameter("car_engine", "models/car.engine")
        self.declare_parameter("armor_engine", "models/armor.engine")
        self.declare_parameter("car_yaml", "yaml/car.yaml")
        self.declare_parameter("armor_yaml", "yaml/armor.yaml")
        self.declare_parameter("car_conf", 0.15)
        self.declare_parameter("car_iou", 0.5)
        self.declare_parameter("armor_conf", 0.45)
        self.declare_parameter("armor_iou", 0.2)
        self.declare_parameter("armor_roi_expand_ratio", 0.12)
        self.declare_parameter("armor_roi_bottom_expand_ratio", 0.18)
        self.declare_parameter("enhance_armor_roi_on_miss", True)
        self.declare_parameter("test_map_path", "")
        self.declare_parameter("test_calib_map_path", "")
        self.declare_parameter("test_array_path", "")
        self.declare_parameter("test_mask_path", "")
        self.declare_parameter("brightness_gamma", 0.7)   # <1 提亮，0.7 轻度
        self.declare_parameter("brightness_bias", 20)      # 亮度偏移
        self.declare_parameter("brightness_clahe", True)  # CLAHE 自适应增强

        self.nyush_path = self.get_parameter("nyush_path").value
        self.map_mode = str(self.get_parameter("map_mode").value).lower()
        self.state = str(self.get_parameter("state").value).upper()
        self.field_width_m = float(self.get_parameter("field_width_m").value)
        self.field_height_m = float(self.get_parameter("field_height_m").value)
        self.calibration_width_px = int(self.get_parameter("calibration_width_px").value)
        self.calibration_height_px = int(self.get_parameter("calibration_height_px").value)
        self.window_size = max(1, int(self.get_parameter("window_size").value))
        self.max_inactive_time = float(self.get_parameter("max_inactive_time").value)
        self.car_conf = float(self.get_parameter("car_conf").value)
        self.armor_conf = float(self.get_parameter("armor_conf").value)
        self.armor_roi_expand_ratio = max(
            0.0, float(self.get_parameter("armor_roi_expand_ratio").value)
        )
        self.armor_roi_bottom_expand_ratio = max(
            0.0, float(self.get_parameter("armor_roi_bottom_expand_ratio").value)
        )
        self.enhance_armor_roi_on_miss = bool(
            self.get_parameter("enhance_armor_roi_on_miss").value
        )
        self.diagnostic_log_every_sec = float(
            self.get_parameter("diagnostic_log_every_sec").value
        )
        self.publish_debug_image = bool(self.get_parameter("publish_debug_image").value)
        self.publish_debug_map = bool(self.get_parameter("publish_debug_map").value)
        self.debug_image_every_n = max(1, int(self.get_parameter("debug_image_every_n").value))
        self.process_width = max(0, int(self.get_parameter("process_width").value))
        self.process_height = max(0, int(self.get_parameter("process_height").value))
        self.process_interval_ms = max(1, int(self.get_parameter("process_interval_ms").value))
        self.brightness_gamma = float(self.get_parameter("brightness_gamma").value)
        self.brightness_bias = int(self.get_parameter("brightness_bias").value)
        self.brightness_clahe = bool(self.get_parameter("brightness_clahe").value)
        self.frame_count = 0
        self._logged_frame_scale = False
        self._last_diag_log_time = 0.0
        self._latest_image_msg = None
        self._processing_image = False
        self.clahe = cv2.createCLAHE(clipLimit=1.5, tileGridSize=(8, 8)) if self.brightness_clahe else None
        self.brightness_lut = self._build_brightness_lut()

        if not os.path.isdir(self.nyush_path):
            raise RuntimeError(f"NYUSH path does not exist: {self.nyush_path}")
        if self.state not in {"R", "B"}:
            raise RuntimeError(f"state must be R or B, got: {self.state}")
        if self.map_mode not in {"battle", "testmap"}:
            raise RuntimeError(f"map_mode must be battle or testmap, got: {self.map_mode}")

        if self.nyush_path not in sys.path:
            sys.path.insert(0, self.nyush_path)

        self._install_tensorrt_compat()

        from detect_function import YOLOv5Detector

        os.chdir(self.nyush_path)

        self.bridge = CvBridge()
        self.filter = SlidingWindowFilter(
            window_size=self.window_size,
            max_inactive_time=self.max_inactive_time,
        )

        self._load_mapping_assets()
        self._apply_calibration_metadata()
        self._validate_mapping_configuration()

        self.car_detector = self._create_detector(
            weights_path=self._resolve_asset_path(self.get_parameter("car_engine").value),
            data=self.get_parameter("car_yaml").value,
            conf_thres=float(self.get_parameter("car_conf").value),
            iou_thres=float(self.get_parameter("car_iou").value),
            max_det=14,
            detector_cls=YOLOv5Detector,
        )
        self.armor_detector = self._create_detector(
            weights_path=self._resolve_asset_path(self.get_parameter("armor_engine").value),
            data=self.get_parameter("armor_yaml").value,
            conf_thres=float(self.get_parameter("armor_conf").value),
            iou_thres=float(self.get_parameter("armor_iou").value),
            max_det=1,
            detector_cls=YOLOv5Detector,
        )

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.image_sub = self.create_subscription(
            Image, "camera_image", self._queue_image_callback, sensor_qos
        )
        self.process_timer = self.create_timer(
            self.process_interval_ms / 1000.0, self._process_latest_image
        )
        self.resolve_pub = self.create_publisher(DetectResult, "resolve_result", sensor_qos)
        self.debug_image_pub = (
            self.create_publisher(Image, "detect_image", sensor_qos)
            if self.publish_debug_image
            else None
        )
        self.debug_map_pub = (
            self.create_publisher(Image, "nyush_map_image", sensor_qos)
            if self.publish_debug_map
            else None
        )

        self.get_logger().info(
            f"NYUSH world node ready: path={self.nyush_path}, map_mode={self.map_mode}, "
            f"state={self.state}, field={self.field_width_m}x{self.field_height_m}m, "
            f"window={self.window_size}, max_inactive={self.max_inactive_time:.1f}s, "
            f"car_conf={self.car_conf:.2f}, armor_conf={self.armor_conf:.2f}, "
            f"armor_roi_expand={self.armor_roi_expand_ratio:.2f}/{self.armor_roi_bottom_expand_ratio:.2f}, "
            f"debug_image={self.publish_debug_image}, debug_every_n={self.debug_image_every_n}, "
            f"process_size={self.process_width or 'camera'}x{self.process_height or 'camera'}, "
            f"process_interval={self.process_interval_ms}ms"
        )

    def _build_brightness_lut(self):
        if self.brightness_gamma == 1.0 and self.brightness_bias == 0:
            return None

        values = np.arange(256, dtype=np.float32)
        if self.brightness_gamma != 1.0:
            inv_g = 1.0 / self.brightness_gamma
            values = np.power(values / 255.0, inv_g) * 255.0
        if self.brightness_bias != 0:
            values = np.clip(values + self.brightness_bias, 0, 255)
        return values.astype(np.uint8)

    def _apply_brightness(self, img: np.ndarray) -> np.ndarray:
        """对 detect_image 做亮度增强：CLAHE + gamma + bias"""
        out = img.copy()
        # CLAHE 自适应直方图均衡（对暗图效果好）
        if self.clahe is not None:
            lab = cv2.cvtColor(out, cv2.COLOR_BGR2LAB)
            l, a, b = cv2.split(lab)
            l = self.clahe.apply(l)
            out = cv2.merge([l, a, b])
            out = cv2.cvtColor(out, cv2.COLOR_LAB2BGR)
        if self.brightness_lut is None:
            return out
        return cv2.LUT(out, self.brightness_lut)

    def _resize_for_processing(self, frame: np.ndarray) -> np.ndarray:
        img_h, img_w = frame.shape[:2]
        target_w = self.process_width
        target_h = self.process_height

        if target_w <= 0 and target_h <= 0:
            return frame
        if target_w > 0 and target_h > 0:
            new_w, new_h = target_w, target_h
        elif target_w > 0:
            new_w = target_w
            new_h = max(1, int(round(img_h * target_w / img_w)))
        else:
            new_h = target_h
            new_w = max(1, int(round(img_w * target_h / img_h)))

        if new_w == img_w and new_h == img_h:
            return frame
        return cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_LINEAR)

    def _queue_image_callback(self, msg: Image):
        self._latest_image_msg = msg

    def _process_latest_image(self):
        if self._processing_image or self._latest_image_msg is None:
            return

        msg = self._latest_image_msg
        self._latest_image_msg = None
        self._processing_image = True
        try:
            self.image_callback(msg)
        finally:
            self._processing_image = False

    def _install_tensorrt_compat(self) -> None:
        if "tensorrt" in sys.modules:
            return
        try:
            importlib.import_module("tensorrt")
            return
        except ModuleNotFoundError:
            pass

        try:
            sys.modules["tensorrt"] = importlib.import_module("tensorrt_dispatch")
            self.get_logger().info(
                "Using TensorRT compatibility import via tensorrt_dispatch."
            )
        except ModuleNotFoundError:
            self.get_logger().warning(
                "TensorRT Python bindings are unavailable; .engine models will not load."
            )

    def _resolve_asset_path(self, configured_path: str) -> str:
        asset_path = Path(str(configured_path).strip())
        if not asset_path.is_absolute():
            asset_path = Path(self.nyush_path) / asset_path
        return str(asset_path)

    def _create_detector(
        self,
        *,
        weights_path: str,
        data: str,
        conf_thres: float,
        iou_thres: float,
        max_det: int,
        detector_cls,
    ):
        try:
            return detector_cls(
                weights_path,
                data=data,
                conf_thres=conf_thres,
                iou_thres=iou_thres,
                max_det=max_det,
                ui=False,
            )
        except Exception as exc:
            onnx_path = str(Path(weights_path).with_suffix(".onnx"))
            if not weights_path.endswith(".engine") or not os.path.exists(onnx_path):
                raise

            self.get_logger().warning(
                "Failed to load TensorRT engine %s (%s). Falling back to ONNX/OpenCV DNN: %s"
                % (weights_path, exc, onnx_path)
            )
            return detector_cls(
                onnx_path,
                data=data,
                conf_thres=conf_thres,
                iou_thres=iou_thres,
                max_det=max_det,
                ui=False,
                dnn=True,
                half=False,
            )

    def _param_path(self, param_name: str, fallback_relative: str) -> str:
        value = str(self.get_parameter(param_name).value).strip()
        if value:
            return value
        return os.path.join(self.nyush_path, fallback_relative)

    @staticmethod
    def _metadata_path_for_array(array_path: str) -> str:
        array_file = Path(array_path)
        return str(array_file.with_suffix(".meta.json"))

    def _load_calibration_metadata(self, array_path: str):
        metadata_path = self._metadata_path_for_array(array_path)
        if not os.path.exists(metadata_path):
            return None
        try:
            with open(metadata_path, "r", encoding="utf-8") as f:
                metadata = json.load(f)
            metadata["_metadata_path"] = metadata_path
            return metadata
        except Exception as exc:
            self.get_logger().warning(
                "Failed to load calibration metadata %s: %s" % (metadata_path, exc)
            )
            return None

    def _apply_calibration_metadata(self) -> None:
        metadata = self.calibration_metadata
        if metadata is None:
            return

        metadata_map_mode = str(metadata.get("map_profile", "")).strip().lower()
        metadata_state = str(metadata.get("state", "")).strip().upper()
        meta_w = metadata.get("calibration_image_width_px")
        meta_h = metadata.get("calibration_image_height_px")
        metadata_path = metadata.get("_metadata_path", "<unknown>")

        self.get_logger().info(
            "Loaded NYUSH calibration metadata: %s" % metadata_path
        )

        if metadata_map_mode and metadata_map_mode != self.map_mode:
            self.get_logger().warning(
                "Calibration metadata map_profile=%s but runtime map_mode=%s"
                % (metadata_map_mode, self.map_mode)
            )
        if metadata_state and metadata_state != self.state:
            self.get_logger().warning(
                "Calibration metadata state=%s but runtime state=%s"
                % (metadata_state, self.state)
            )

        if meta_w is None or meta_h is None:
            self.get_logger().warning(
                "Calibration metadata is missing calibration_image_width_px/height_px"
            )
            return

        meta_w = int(meta_w)
        meta_h = int(meta_h)
        if self.calibration_width_px <= 0:
            self.calibration_width_px = meta_w
        elif self.calibration_width_px != meta_w:
            self.get_logger().warning(
                "Launch calibration_width_px=%d overrides metadata width=%d"
                % (self.calibration_width_px, meta_w)
            )

        if self.calibration_height_px <= 0:
            self.calibration_height_px = meta_h
        elif self.calibration_height_px != meta_h:
            self.get_logger().warning(
                "Launch calibration_height_px=%d overrides metadata height=%d"
                % (self.calibration_height_px, meta_h)
            )

    def _load_mapping_assets(self) -> None:
        calib_map_path = ""
        if self.map_mode == "battle":
            if self.state == "R":
                array_path = os.path.join(self.nyush_path, "arrays_test_red.npy")
                map_path = os.path.join(self.nyush_path, "images/map_red.jpg")
            else:
                array_path = os.path.join(self.nyush_path, "arrays_test_blue.npy")
                map_path = os.path.join(self.nyush_path, "images/map_blue.jpg")
            mask_path = os.path.join(self.nyush_path, "images/map_mask.jpg")
        else:
            array_path = self._param_path("test_array_path", "array_test_custom.npy")
            map_path = self._param_path("test_map_path", "images/my_map(m).jpg")
            calib_map_path = self._param_path("test_calib_map_path", "images/my_map.jpg")
            mask_path = str(self.get_parameter("test_mask_path").value).strip()

        self.calibration_metadata = self._load_calibration_metadata(array_path)
        if self.calibration_metadata is not None:
            metadata_map_path = str(self.calibration_metadata.get("runtime_map_path", "")).strip()
            metadata_calib_map_path = str(
                self.calibration_metadata.get("calibration_map_path", "")
            ).strip()
            if self.map_mode == "testmap":
                if not str(self.get_parameter("test_map_path").value).strip() and metadata_map_path:
                    map_path = metadata_map_path
                if (
                    not str(self.get_parameter("test_calib_map_path").value).strip()
                    and metadata_calib_map_path
                ):
                    calib_map_path = metadata_calib_map_path
        self.loaded_arrays = np.load(array_path, allow_pickle=True)
        self.map_image = cv2.imread(map_path)
        if self.map_image is None:
            raise RuntimeError(f"Failed to load map image: {map_path}")
        self.test_calib_width = None
        self.test_calib_height = None
        if calib_map_path:
            calib_map_image = cv2.imread(calib_map_path)
            if calib_map_image is None:
                raise RuntimeError(f"Failed to load calibration map image: {calib_map_path}")
            self.test_calib_height = calib_map_image.shape[0]
            self.test_calib_width = calib_map_image.shape[1]

        if mask_path:
            self.mask_image = cv2.imread(mask_path)
            if self.mask_image is None:
                raise RuntimeError(f"Failed to load mask image: {mask_path}")
        else:
            self.mask_image = np.zeros_like(self.map_image)

        self.map_height, self.map_width = self.map_image.shape[:2]
        self.mask_height = self.mask_image.shape[0] - 1
        self.mask_width = self.mask_image.shape[1] - 1

        self.m_ground = self.loaded_arrays[0]
        self.m_height_r = self.loaded_arrays[1] if len(self.loaded_arrays) > 1 else self.m_ground
        self.m_height_g = self.loaded_arrays[2] if len(self.loaded_arrays) > 2 else self.m_ground

    def _validate_mapping_configuration(self) -> None:
        map_aspect = float(self.map_width) / float(self.map_height)
        inverse_map_aspect = float(self.map_height) / float(self.map_width)
        field_aspect = float(self.field_width_m) / float(self.field_height_m)
        aspect_error = abs(field_aspect - map_aspect) / map_aspect

        if self.map_mode == "testmap":
            calib_desc = "unknown"
            if self.test_calib_width is not None and self.test_calib_height is not None:
                calib_desc = f"{self.test_calib_width}x{self.test_calib_height}"
            self.get_logger().info(
                "NYUSH testmap assets: runtime_map=%dx%d, calibration_map=%s, "
                "expected_testmap_field=%.2fx%.2fm"
                % (
                    self.map_width,
                    self.map_height,
                    calib_desc,
                    TESTMAP_FIELD_WIDTH_M,
                    TESTMAP_FIELD_HEIGHT_M,
                )
            )
            if self.field_width_m < self.field_height_m:
                self.get_logger().warning(
                    "testmap field_width_m=%.2f is smaller than field_height_m=%.2f, "
                    "but my_map(m).jpg is landscape after rotation. "
                    "This usually means width/height were swapped."
                    % (self.field_width_m, self.field_height_m)
                )
            inverse_aspect_error = abs(field_aspect - inverse_map_aspect) / inverse_map_aspect
            if inverse_aspect_error <= 0.10:
                self.get_logger().warning(
                    "testmap field aspect %.3f matches the portrait calibration-map aspect %.3f "
                    "instead of the runtime map aspect %.3f. "
                    "For my_map(m).jpg you likely want field_width_m=%.2f and field_height_m=%.2f."
                    % (
                        field_aspect,
                        inverse_map_aspect,
                        map_aspect,
                        TESTMAP_FIELD_WIDTH_M,
                        TESTMAP_FIELD_HEIGHT_M,
                    )
                )

        if aspect_error > 0.10:
            self.get_logger().warning(
                "Map metric aspect ratio mismatch: map=%dx%d (%.3f), field=%.2fx%.2fm (%.3f). "
                "World coordinates will be skewed; check field_width_m/field_height_m for the active map."
                % (
                    self.map_width,
                    self.map_height,
                    map_aspect,
                    self.field_width_m,
                    self.field_height_m,
                    field_aspect,
                )
            )

    def _mapped_point_to_display(self, mapped_point: np.ndarray):
        map_x = float(mapped_point[0][0][0])
        map_y = float(mapped_point[0][0][1])
        if self.map_mode != "testmap":
            return map_x, map_y
        if self.test_calib_width is None:
            raise RuntimeError("testmap calibration width is unavailable")
        # NYUSH testmap calibration is performed on my_map.jpg, while runtime display
        # uses the rotated my_map(m).jpg asset.
        return map_y, float((self.test_calib_width - 1) - map_x)

    def _clamp_display_point(self, mapped_point: np.ndarray):
        display_x, display_y = self._mapped_point_to_display(mapped_point)
        x_c = max(0, min(int(display_x), self.mask_width))
        y_c = max(0, min(int(display_y), self.mask_height))
        return x_c, y_c

    def _class_to_index(self, cls_name: str):
        if len(cls_name) < 2:
            return None
        try:
            robot_num = int(cls_name[1])
        except ValueError:
            return None
        if 1 <= robot_num <= 6:
            return robot_num - 1
        if robot_num == 7:
            return 5
        return None

    def _add_map_point(self, cls_name: str, camera_point: np.ndarray):
        mapped_point = cv2.perspectiveTransform(camera_point.reshape(1, 1, 2), self.m_ground)
        x_c, y_c = self._clamp_display_point(mapped_point)
        color = self.mask_image[y_c, x_c]
        if int(color[0]) == int(color[1]) == int(color[2]) == 0:
            self.filter.add_data(cls_name, x_c, y_c)
            return

        mapped_point = cv2.perspectiveTransform(camera_point.reshape(1, 1, 2), self.m_height_r)
        x_c, y_c = self._clamp_display_point(mapped_point)
        color = self.mask_image[y_c, x_c]
        if int(color[1]) > int(color[0]) and int(color[1]) > int(color[2]):
            self.filter.add_data(cls_name, x_c, y_c)
            return

        mapped_point = cv2.perspectiveTransform(camera_point.reshape(1, 1, 2), self.m_height_g)
        x_c, y_c = self._clamp_display_point(mapped_point)
        color = self.mask_image[y_c, x_c]
        if int(color[0]) > int(color[1]) and int(color[0]) > int(color[2]):
            self.filter.add_data(cls_name, x_c, y_c)

    def _map_pixel_to_world(self, pixel_x: float, pixel_y: float):
        if self.map_mode == "testmap":
            world_x_px = pixel_x
            world_y_px = self.map_height - pixel_y
        elif self.state == "R":
            world_x_px = self.map_width - pixel_y
            world_y_px = self.map_height - pixel_x
        else:
            world_x_px = pixel_y
            world_y_px = pixel_x

        world_x_m = float(world_x_px) * self.field_width_m / float(self.map_width)
        world_y_m = float(world_y_px) * self.field_height_m / float(self.map_height)
        return world_x_m, world_y_m

    def _publish_debug_map(self, world_result: DetectResult):
        if self.debug_map_pub is None:
            return

        debug_map = self.map_image.copy()
        scale_x = float(self.map_width) / self.field_width_m
        scale_y = float(self.map_height) / self.field_height_m

        def draw_points(xs, ys, color, prefix):
            for idx, (x_m, y_m) in enumerate(zip(xs, ys), start=1):
                if x_m == 0.0 and y_m == 0.0:
                    continue
                px = int(x_m * scale_x)
                py = int(self.map_height - y_m * scale_y)
                cv2.circle(debug_map, (px, py), 8, color, -1)
                cv2.putText(
                    debug_map,
                    f"{prefix}{7 if idx == 6 else idx}",
                    (px + 6, py - 6),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    color,
                    2,
                )

        draw_points(world_result.red_x, world_result.red_y, (0, 0, 255), "R")
        draw_points(world_result.blue_x, world_result.blue_y, (255, 0, 0), "B")

        map_msg = self.bridge.cv2_to_imgmsg(debug_map, "bgr8")
        self.debug_map_pub.publish(map_msg)

    def _expand_car_roi(self, x: float, y: float, w: float, h: float, img_w: int, img_h: int):
        margin_x = int(round(w * self.armor_roi_expand_ratio))
        margin_top = int(round(h * self.armor_roi_expand_ratio))
        margin_bottom = int(round(h * self.armor_roi_bottom_expand_ratio))
        x1 = max(0, int(round(x)) - margin_x)
        y1 = max(0, int(round(y)) - margin_top)
        x2 = min(img_w, int(round(x + w)) + margin_x)
        y2 = min(img_h, int(round(y + h)) + margin_bottom)
        return x1, y1, x2, y2

    def _predict_armor_detections(self, car_img: np.ndarray):
        armor_detections = self.armor_detector.predict(car_img)
        retry_hit = False
        if armor_detections or not self.enhance_armor_roi_on_miss:
            return armor_detections, retry_hit

        enhanced_img = self._apply_brightness(car_img)
        armor_detections = self.armor_detector.predict(enhanced_img)
        retry_hit = bool(armor_detections)
        return armor_detections, retry_hit

    @staticmethod
    def _count_nonzero_targets(world_result: DetectResult) -> int:
        count = 0
        for xs, ys in (
            (world_result.red_x, world_result.red_y),
            (world_result.blue_x, world_result.blue_y),
        ):
            for x_m, y_m in zip(xs, ys):
                if x_m != 0.0 or y_m != 0.0:
                    count += 1
        return count

    def _maybe_log_detection_status(
        self,
        *,
        car_count: int,
        armor_count: int,
        armor_retry_hits: int,
        filtered_tracks,
        world_result: DetectResult,
    ) -> None:
        if self.diagnostic_log_every_sec <= 0:
            return
        now = time.monotonic()
        if now - self._last_diag_log_time < self.diagnostic_log_every_sec:
            return

        tracks_total = len(filtered_tracks)
        tracks_ready = sum(1 for point in filtered_tracks.values() if point is not None)
        tracks_pending = tracks_total - tracks_ready
        resolve_nonzero = self._count_nonzero_targets(world_result)

        status_msg = (
            "NYUSH status: cars=%d, armors=%d, armor_retry_hits=%d, tracks_ready=%d, tracks_pending=%d, "
            "resolve_nonzero=%d, window_size=%d, max_inactive_time=%.1fs, car_conf=%.2f, armor_conf=%.2f"
            % (
                car_count,
                armor_count,
                armor_retry_hits,
                tracks_ready,
                tracks_pending,
                resolve_nonzero,
                self.window_size,
                self.max_inactive_time,
                self.car_conf,
                self.armor_conf,
            )
        )

        if car_count > 0 and armor_count == 0:
            self.get_logger().warning(
                status_msg + " | car boxes exist but armor detector returned 0"
            )
        elif armor_count > 0 and tracks_ready == 0:
            self.get_logger().warning(
                status_msg + " | armor detections exist but SlidingWindowFilter has not filled yet"
            )
        elif armor_count > 0 and resolve_nonzero == 0:
            self.get_logger().warning(
                status_msg + " | armors detected but resolve_result is still all zero; check class names or calibration"
            )
        else:
            self.get_logger().info(status_msg)

        self._last_diag_log_time = now

    def image_callback(self, msg: Image):
        try:
            self.frame_count += 1
            should_publish_debug = (
                self.debug_image_pub is not None
                and self.frame_count % self.debug_image_every_n == 0
            )
            source_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            raw_h, raw_w = source_frame.shape[:2]
            frame = self._resize_for_processing(source_frame)
            debug_frame = frame.copy() if should_publish_debug else None
            img_h, img_w = frame.shape[:2]
            calib_w = float(self.calibration_width_px) if self.calibration_width_px > 0 else float(raw_w)
            calib_h = float(self.calibration_height_px) if self.calibration_height_px > 0 else float(raw_h)
            scale_x = calib_w / float(img_w) if img_w else 1.0
            scale_y = calib_h / float(img_h) if img_h else 1.0
            raw_scale_x = calib_w / float(raw_w) if raw_w else 1.0
            raw_scale_y = calib_h / float(raw_h) if raw_h else 1.0

            if not self._logged_frame_scale:
                self.get_logger().info(
                    "NYUSH calibration scaling: camera_frame=%dx%d, process_frame=%dx%d, calibration_frame=%.1fx%.1f, "
                    "camera_scale=(%.3f, %.3f), process_scale=(%.3f, %.3f)"
                    % (raw_w, raw_h, img_w, img_h, calib_w, calib_h, raw_scale_x, raw_scale_y, scale_x, scale_y)
                )
                if abs(raw_scale_x - 1.0) > 0.05 or abs(raw_scale_y - 1.0) > 0.05:
                    self.get_logger().warning(
                        "Calibration frame size does not match runtime camera_image size. "
                        "If calibration.py was run on the current stream, pass calibration_width_px/height_px "
                        "to match that stream or the mapped world coordinates will be far off."
                    )
                self._logged_frame_scale = True

            car_count = 0
            armor_count = 0
            armor_retry_hits = 0
            car_detections = self.car_detector.predict(frame)
            for det in car_detections:
                cls_name, (x, y, w, h), conf = det
                if cls_name != "car":
                    continue
                car_count += 1

                x1, y1, x2, y2 = self._expand_car_roi(x, y, w, h, img_w, img_h)
                if x2 <= x1 or y2 <= y1:
                    continue

                car_img = frame[y1:y2, x1:x2]
                armor_detections, retry_hit = self._predict_armor_detections(car_img)
                if retry_hit:
                    armor_retry_hits += 1

                if should_publish_debug:
                    cv2.rectangle(debug_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(
                        debug_frame,
                        f"car:{conf:.2f}",
                        (x1, max(20, y1 - 10)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.8,
                        (0, 255, 0),
                        2,
                    )

                for armor_det in armor_detections:
                    armor_cls, (ax, ay, aw, ah), armor_conf = armor_det
                    if not armor_cls:
                        continue
                    armor_count += 1

                    center_x = min(x1 + ax + aw / 2.0, img_w - 1)
                    center_y = min(y1 + ay + 1.5 * ah, img_h - 1)
                    camera_point = np.array(
                        [[[center_x * scale_x, center_y * scale_y]]], dtype=np.float32
                    )
                    self._add_map_point(armor_cls, camera_point)

                    if should_publish_debug:
                        label_color = (255, 0, 0) if armor_cls.startswith("B") else (0, 0, 255)
                        cv2.putText(
                            debug_frame,
                            f"{armor_cls}:{armor_conf:.2f}",
                            (x1, min(img_h - 10, y2 + 25)),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.8,
                            label_color,
                            2,
                        )
                        cv2.circle(debug_frame, (int(center_x), int(center_y)), 4, label_color, -1)

            world_result = DetectResult()
            world_result.header.stamp = msg.header.stamp
            world_result.blue_x = [0.0] * 6
            world_result.blue_y = [0.0] * 6
            world_result.red_x = [0.0] * 6
            world_result.red_y = [0.0] * 6

            filtered_tracks = self.filter.get_all_data()
            for cls_name, pixel_xy in filtered_tracks.items():
                if pixel_xy is None:
                    continue
                index = self._class_to_index(cls_name)
                if index is None:
                    continue

                world_x_m, world_y_m = self._map_pixel_to_world(pixel_xy[0], pixel_xy[1])
                if cls_name.startswith("B"):
                    world_result.blue_x[index] = world_x_m
                    world_result.blue_y[index] = world_y_m
                elif cls_name.startswith("R"):
                    world_result.red_x[index] = world_x_m
                    world_result.red_y[index] = world_y_m

            self.resolve_pub.publish(world_result)
            self._maybe_log_detection_status(
                car_count=car_count,
                armor_count=armor_count,
                armor_retry_hits=armor_retry_hits,
                filtered_tracks=filtered_tracks,
                world_result=world_result,
            )

            if should_publish_debug:
                out_frame = self._apply_brightness(debug_frame)
                debug_msg = self.bridge.cv2_to_imgmsg(out_frame, "bgr8")
                debug_msg.header = msg.header
                self.debug_image_pub.publish(debug_msg)

            self._publish_debug_map(world_result)
        except Exception as exc:
            self.get_logger().error(f"NYUSH world callback failed: {exc}")


def main(args=None):
    rclpy.init(args=args)
    node = NYUSHWorldNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
