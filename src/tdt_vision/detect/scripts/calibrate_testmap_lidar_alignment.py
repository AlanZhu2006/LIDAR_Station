#!/usr/bin/env python3
"""
Manual similarity calibration between the NYUSH testmap frame and LiDAR rm_frame.

This tool:
1. Loads the runtime NYUSH test map image (the same base image used by /nyush_map_image).
2. Projects a LiDAR PCD map into a top-down occupancy-style image.
3. Lets the operator click corresponding landmarks on both maps.
4. Solves a 2D similarity transform from testmap-world meters -> rm_frame meters.
5. Saves the transform YAML plus preview assets for runtime use.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import yaml

try:
    import open3d as o3d
except ImportError as exc:  # pragma: no cover - dependency guard
    raise SystemExit(
        "open3d is required for calibrate_testmap_lidar_alignment.py. "
        "Install it first or use the existing environment where rotate_pcd.py works."
    ) from exc


DEFAULT_FIELD_WIDTH_M = 6.79
DEFAULT_FIELD_HEIGHT_M = 3.82
DEFAULT_RESOLUTION_M_PER_PX = 0.04
DEFAULT_Z_MIN_M = -1.0
DEFAULT_Z_MAX_M = 2.5
DEFAULT_PADDING_M = 0.4
WINDOW_NAME = "Testmap <-> LiDAR Alignment"
TOP_TEXT_HEIGHT = 150
PANEL_GAP = 24
LEFT_PANEL_MAX_W = 900
RIGHT_PANEL_MAX_W = 900
PANEL_MAX_H = 760


@dataclass
class TopDownMeta:
    x_min_m: float
    x_max_m: float
    y_min_m: float
    y_max_m: float
    resolution_m_per_px: float
    width_px: int
    height_px: int
    z_min_m: float
    z_max_m: float
    padding_m: float
    point_count: int


def ensure_parent(path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)


def testmap_pixel_to_world(
    pixel_x: float,
    pixel_y: float,
    map_width_px: int,
    map_height_px: int,
    field_width_m: float,
    field_height_m: float,
) -> np.ndarray:
    world_x_m = float(pixel_x) * field_width_m / float(map_width_px)
    world_y_m = float(map_height_px - pixel_y) * field_height_m / float(map_height_px)
    return np.array([world_x_m, world_y_m], dtype=np.float64)


def topdown_pixel_to_world(pixel_x: float, pixel_y: float, meta: TopDownMeta) -> np.ndarray:
    world_x_m = meta.x_min_m + float(pixel_x) * meta.resolution_m_per_px
    world_y_m = meta.y_min_m + (meta.height_px - 1 - float(pixel_y)) * meta.resolution_m_per_px
    return np.array([world_x_m, world_y_m], dtype=np.float64)


def similarity_matrix_to_components(matrix_2x3: np.ndarray) -> tuple[float, float, float, float]:
    a, b, tx = matrix_2x3[0]
    c, d, ty = matrix_2x3[1]
    scale = float(math.sqrt(a * a + c * c))
    rotation_rad = float(math.atan2(c, a))
    return scale, rotation_rad, float(tx), float(ty)


def build_testmap_pixel_to_world_affine(
    map_width_px: int,
    map_height_px: int,
    field_width_m: float,
    field_height_m: float,
) -> np.ndarray:
    scale_x = field_width_m / float(map_width_px)
    scale_y = field_height_m / float(map_height_px)
    return np.array(
        [
            [scale_x, 0.0, 0.0],
            [0.0, -scale_y, field_height_m],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )


def build_world_to_topdown_pixel_affine(meta: TopDownMeta) -> np.ndarray:
    resolution = meta.resolution_m_per_px
    return np.array(
        [
            [1.0 / resolution, 0.0, -meta.x_min_m / resolution],
            [0.0, -1.0 / resolution, meta.height_px - 1 + meta.y_min_m / resolution],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )


def render_lidar_topdown(
    pcd_path: Path,
    *,
    resolution_m_per_px: float,
    z_min_m: float,
    z_max_m: float,
    padding_m: float,
) -> tuple[np.ndarray, TopDownMeta]:
    point_cloud = o3d.io.read_point_cloud(str(pcd_path))
    points = np.asarray(point_cloud.points, dtype=np.float64)
    if points.size == 0:
        raise RuntimeError(f"PCD has no points: {pcd_path}")

    finite_mask = np.isfinite(points).all(axis=1)
    points = points[finite_mask]
    z_mask = (points[:, 2] >= z_min_m) & (points[:, 2] <= z_max_m)
    filtered = points[z_mask]
    if filtered.size == 0:
        raise RuntimeError(
            f"No points remain after z filter [{z_min_m}, {z_max_m}] for {pcd_path}"
        )

    x_min = float(filtered[:, 0].min() - padding_m)
    x_max = float(filtered[:, 0].max() + padding_m)
    y_min = float(filtered[:, 1].min() - padding_m)
    y_max = float(filtered[:, 1].max() + padding_m)
    width_px = int(math.ceil((x_max - x_min) / resolution_m_per_px)) + 1
    height_px = int(math.ceil((y_max - y_min) / resolution_m_per_px)) + 1

    x_idx = np.floor((filtered[:, 0] - x_min) / resolution_m_per_px).astype(np.int32)
    y_idx = np.floor((filtered[:, 1] - y_min) / resolution_m_per_px).astype(np.int32)
    x_idx = np.clip(x_idx, 0, width_px - 1)
    y_idx = np.clip(y_idx, 0, height_px - 1)

    grid = np.zeros((height_px, width_px), dtype=np.float32)
    np.add.at(grid, (y_idx, x_idx), 1.0)
    grid = np.flipud(grid)
    grid = np.log1p(grid)
    if float(grid.max()) > 0.0:
        grid = grid / float(grid.max())
    grid = cv2.GaussianBlur(grid, (0, 0), sigmaX=1.2, sigmaY=1.2)
    grid_u8 = np.clip(grid * 255.0, 0.0, 255.0).astype(np.uint8)
    grid_u8 = cv2.equalizeHist(grid_u8)
    topdown_bgr = cv2.cvtColor(grid_u8, cv2.COLOR_GRAY2BGR)

    meta = TopDownMeta(
        x_min_m=x_min,
        x_max_m=x_max,
        y_min_m=y_min,
        y_max_m=y_max,
        resolution_m_per_px=resolution_m_per_px,
        width_px=width_px,
        height_px=height_px,
        z_min_m=z_min_m,
        z_max_m=z_max_m,
        padding_m=padding_m,
        point_count=int(filtered.shape[0]),
    )
    return topdown_bgr, meta


def resize_for_panel(image: np.ndarray, max_w: int, max_h: int) -> tuple[np.ndarray, float]:
    height, width = image.shape[:2]
    scale = min(max_w / float(width), max_h / float(height))
    resized = cv2.resize(
        image,
        (max(1, int(round(width * scale))), max(1, int(round(height * scale)))),
        interpolation=cv2.INTER_AREA if scale < 1.0 else cv2.INTER_LINEAR,
    )
    return resized, scale


class ManualAlignmentUI:
    def __init__(
        self,
        *,
        testmap_bgr: np.ndarray,
        lidar_topdown_bgr: np.ndarray,
        testmap_path: Path,
        pcd_path: Path,
        output_yaml_path: Path,
        topdown_png_path: Path,
        preview_png_path: Path,
        field_width_m: float,
        field_height_m: float,
        lidar_meta: TopDownMeta,
    ):
        self.testmap_original = testmap_bgr
        self.lidar_original = lidar_topdown_bgr
        self.testmap_path = testmap_path
        self.pcd_path = pcd_path
        self.output_yaml_path = output_yaml_path
        self.topdown_png_path = topdown_png_path
        self.preview_png_path = preview_png_path
        self.field_width_m = field_width_m
        self.field_height_m = field_height_m
        self.lidar_meta = lidar_meta

        self.testmap_panel, self.testmap_scale = resize_for_panel(
            self.testmap_original, LEFT_PANEL_MAX_W, PANEL_MAX_H
        )
        self.lidar_panel, self.lidar_scale = resize_for_panel(
            self.lidar_original, RIGHT_PANEL_MAX_W, PANEL_MAX_H
        )
        self.left_w = self.testmap_panel.shape[1]
        self.right_w = self.lidar_panel.shape[1]
        self.panel_h = max(self.testmap_panel.shape[0], self.lidar_panel.shape[0])
        self.canvas_w = self.left_w + PANEL_GAP + self.right_w
        self.canvas_h = TOP_TEXT_HEIGHT + self.panel_h

        self.pending_testmap_px: np.ndarray | None = None
        self.pending_lidar_px: np.ndarray | None = None
        self.pairs: list[dict[str, np.ndarray]] = []
        self.saved_result: dict | None = None

    def _screen_to_testmap_px(self, x: int, y: int) -> np.ndarray:
        rel_x = np.clip(x, 0, self.left_w - 1)
        rel_y = np.clip(y - TOP_TEXT_HEIGHT, 0, self.testmap_panel.shape[0] - 1)
        return np.array([rel_x / self.testmap_scale, rel_y / self.testmap_scale], dtype=np.float64)

    def _screen_to_lidar_px(self, x: int, y: int) -> np.ndarray:
        rel_x = np.clip(x - self.left_w - PANEL_GAP, 0, self.right_w - 1)
        rel_y = np.clip(y - TOP_TEXT_HEIGHT, 0, self.lidar_panel.shape[0] - 1)
        return np.array([rel_x / self.lidar_scale, rel_y / self.lidar_scale], dtype=np.float64)

    def _pair_ready(self) -> bool:
        return self.pending_testmap_px is not None and self.pending_lidar_px is not None

    def _commit_pending_pair(self) -> None:
        if not self._pair_ready():
            return
        self.pairs.append(
            {
                "testmap_px": self.pending_testmap_px.copy(),
                "lidar_px": self.pending_lidar_px.copy(),
            }
        )
        self.pending_testmap_px = None
        self.pending_lidar_px = None

    def _undo_last(self) -> None:
        if self.pairs:
            self.pairs.pop()
            return
        if self.pending_lidar_px is not None:
            self.pending_lidar_px = None
            return
        if self.pending_testmap_px is not None:
            self.pending_testmap_px = None

    def _reset(self) -> None:
        self.pending_testmap_px = None
        self.pending_lidar_px = None
        self.pairs.clear()
        self.saved_result = None

    def _handle_click(self, event: int, x: int, y: int, _flags: int, _param) -> None:
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        if y < TOP_TEXT_HEIGHT:
            return
        if x < self.left_w:
            self.pending_testmap_px = self._screen_to_testmap_px(x, y)
        elif x >= self.left_w + PANEL_GAP:
            self.pending_lidar_px = self._screen_to_lidar_px(x, y)
        if self._pair_ready():
            self._commit_pending_pair()

    def _draw_marker(
        self,
        image: np.ndarray,
        point_px: np.ndarray,
        *,
        scale: float,
        label: str,
        color: tuple[int, int, int],
        radius: int = 6,
        thickness: int = 2,
    ) -> None:
        x = int(round(point_px[0] * scale))
        y = int(round(point_px[1] * scale))
        cv2.circle(image, (x, y), radius, color, thickness)
        cv2.putText(
            image,
            label,
            (x + 8, y - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            color,
            2,
            cv2.LINE_AA,
        )

    def _render_canvas(self) -> np.ndarray:
        canvas = np.full((self.canvas_h, self.canvas_w, 3), 28, dtype=np.uint8)

        canvas[TOP_TEXT_HEIGHT : TOP_TEXT_HEIGHT + self.testmap_panel.shape[0], 0 : self.left_w] = (
            self.testmap_panel
        )
        right_x0 = self.left_w + PANEL_GAP
        canvas[
            TOP_TEXT_HEIGHT : TOP_TEXT_HEIGHT + self.lidar_panel.shape[0],
            right_x0 : right_x0 + self.right_w,
        ] = self.lidar_panel

        cv2.putText(
            canvas,
            "Left: NYUSH runtime map (/nyush_map_image base) | Right: LiDAR top-down map (rm_frame)",
            (18, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.72,
            (220, 220, 220),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            canvas,
            "Left-click a landmark on each side to create a pair. Keys: s=solve/save, u=undo, r=reset, q=quit",
            (18, 58),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (210, 210, 210),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            canvas,
            f"Pairs: {len(self.pairs)}  |  Need >= 3  |  Output: {self.output_yaml_path}",
            (18, 88),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (160, 255, 160),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            canvas,
            f"PCD: {self.pcd_path.name}  |  Top-down resolution: {self.lidar_meta.resolution_m_per_px:.3f} m/px",
            (18, 118),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (160, 210, 255),
            2,
            cv2.LINE_AA,
        )

        testmap_layer = canvas[
            TOP_TEXT_HEIGHT : TOP_TEXT_HEIGHT + self.testmap_panel.shape[0], 0 : self.left_w
        ]
        lidar_layer = canvas[
            TOP_TEXT_HEIGHT : TOP_TEXT_HEIGHT + self.lidar_panel.shape[0],
            right_x0 : right_x0 + self.right_w,
        ]

        for idx, pair in enumerate(self.pairs, start=1):
            hue = (53 * idx) % 255
            color = tuple(int(c) for c in cv2.cvtColor(
                np.uint8([[[hue, 220, 255]]]), cv2.COLOR_HSV2BGR
            )[0, 0])
            self._draw_marker(
                testmap_layer,
                pair["testmap_px"],
                scale=self.testmap_scale,
                label=str(idx),
                color=color,
            )
            self._draw_marker(
                lidar_layer,
                pair["lidar_px"],
                scale=self.lidar_scale,
                label=str(idx),
                color=color,
            )

        if self.pending_testmap_px is not None:
            self._draw_marker(
                testmap_layer,
                self.pending_testmap_px,
                scale=self.testmap_scale,
                label="P",
                color=(0, 255, 255),
                radius=8,
            )
        if self.pending_lidar_px is not None:
            self._draw_marker(
                lidar_layer,
                self.pending_lidar_px,
                scale=self.lidar_scale,
                label="P",
                color=(0, 255, 255),
                radius=8,
            )

        if self.saved_result is not None:
            error = self.saved_result["error"]
            cv2.putText(
                canvas,
                "Saved: rmse={:.3f}m max={:.3f}m scale={:.4f} rot={:.2f}deg".format(
                    error["rmse_m"],
                    error["max_m"],
                    self.saved_result["similarity_transform"]["scale"],
                    self.saved_result["similarity_transform"]["rotation_deg"],
                ),
                (18, self.canvas_h - 18),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.72,
                (80, 255, 120),
                2,
                cv2.LINE_AA,
            )

        return canvas

    def _solve_and_save(self) -> None:
        if len(self.pairs) < 3:
            raise RuntimeError("Need at least 3 correspondence pairs for similarity calibration.")

        testmap_world = []
        lidar_world = []
        correspondence_entries = []
        for pair in self.pairs:
            src_world = testmap_pixel_to_world(
                pair["testmap_px"][0],
                pair["testmap_px"][1],
                self.testmap_original.shape[1],
                self.testmap_original.shape[0],
                self.field_width_m,
                self.field_height_m,
            )
            dst_world = topdown_pixel_to_world(
                pair["lidar_px"][0], pair["lidar_px"][1], self.lidar_meta
            )
            testmap_world.append(src_world)
            lidar_world.append(dst_world)
            correspondence_entries.append(
                {
                    "testmap_pixel": [float(pair["testmap_px"][0]), float(pair["testmap_px"][1])],
                    "testmap_world_m": [float(src_world[0]), float(src_world[1])],
                    "rm_frame_pixel": [float(pair["lidar_px"][0]), float(pair["lidar_px"][1])],
                    "rm_frame_world_m": [float(dst_world[0]), float(dst_world[1])],
                }
            )

        src = np.asarray(testmap_world, dtype=np.float64)
        dst = np.asarray(lidar_world, dtype=np.float64)
        matrix_2x3, inlier_mask = cv2.estimateAffinePartial2D(
            src.reshape(-1, 1, 2),
            dst.reshape(-1, 1, 2),
            method=cv2.RANSAC,
            ransacReprojThreshold=0.25,
            maxIters=5000,
            confidence=0.999,
            refineIters=100,
        )
        if matrix_2x3 is None:
            raise RuntimeError("Failed to solve similarity transform. Try different landmarks.")

        aligned = (matrix_2x3[:, :2] @ src.T).T + matrix_2x3[:, 2]
        errors = np.linalg.norm(aligned - dst, axis=1)
        rmse_m = float(np.sqrt(np.mean(np.square(errors))))
        max_m = float(np.max(errors))

        scale, rotation_rad, tx, ty = similarity_matrix_to_components(matrix_2x3)
        matrix_3x3 = np.eye(3, dtype=np.float64)
        matrix_3x3[:2, :] = matrix_2x3

        topdown_affine = build_world_to_topdown_pixel_affine(self.lidar_meta)
        testmap_affine = build_testmap_pixel_to_world_affine(
            self.testmap_original.shape[1],
            self.testmap_original.shape[0],
            self.field_width_m,
            self.field_height_m,
        )
        preview_matrix = topdown_affine @ matrix_3x3 @ testmap_affine
        warped_testmap = cv2.warpPerspective(
            self.testmap_original,
            preview_matrix,
            (self.lidar_meta.width_px, self.lidar_meta.height_px),
            flags=cv2.INTER_LINEAR,
        )
        preview = cv2.addWeighted(self.lidar_original, 0.65, warped_testmap, 0.35, 0.0)

        ensure_parent(self.output_yaml_path)
        ensure_parent(self.topdown_png_path)
        ensure_parent(self.preview_png_path)
        cv2.imwrite(str(self.topdown_png_path), self.lidar_original)
        cv2.imwrite(str(self.preview_png_path), preview)

        result = {
            "format_version": 1,
            "saved_at_epoch_s": time.time(),
            "source_frame": "testmap_metric",
            "target_frame": "rm_frame",
            "testmap": {
                "image_path": str(self.testmap_path.resolve()),
                "image_width_px": int(self.testmap_original.shape[1]),
                "image_height_px": int(self.testmap_original.shape[0]),
                "field_width_m": float(self.field_width_m),
                "field_height_m": float(self.field_height_m),
            },
            "lidar_topdown": {
                "pcd_path": str(self.pcd_path.resolve()),
                "image_path": str(self.topdown_png_path.resolve()),
                "preview_path": str(self.preview_png_path.resolve()),
                "x_min_m": float(self.lidar_meta.x_min_m),
                "x_max_m": float(self.lidar_meta.x_max_m),
                "y_min_m": float(self.lidar_meta.y_min_m),
                "y_max_m": float(self.lidar_meta.y_max_m),
                "resolution_m_per_px": float(self.lidar_meta.resolution_m_per_px),
                "width_px": int(self.lidar_meta.width_px),
                "height_px": int(self.lidar_meta.height_px),
                "z_min_m": float(self.lidar_meta.z_min_m),
                "z_max_m": float(self.lidar_meta.z_max_m),
                "padding_m": float(self.lidar_meta.padding_m),
                "point_count": int(self.lidar_meta.point_count),
            },
            "similarity_transform": {
                "scale": float(scale),
                "rotation_rad": float(rotation_rad),
                "rotation_deg": float(np.degrees(rotation_rad)),
                "translation_x_m": float(tx),
                "translation_y_m": float(ty),
                "matrix_3x3": matrix_3x3.tolist(),
            },
            "error": {
                "rmse_m": rmse_m,
                "max_m": max_m,
                "mean_m": float(np.mean(errors)),
                "num_pairs": int(len(self.pairs)),
                "num_inliers": int(np.count_nonzero(inlier_mask)) if inlier_mask is not None else int(len(self.pairs)),
            },
            "correspondences": correspondence_entries,
        }

        with self.output_yaml_path.open("w", encoding="utf-8") as f:
            yaml.safe_dump(result, f, sort_keys=False)

        self.saved_result = result
        print(f"Saved alignment YAML: {self.output_yaml_path}")
        print(f"Saved LiDAR top-down image: {self.topdown_png_path}")
        print(f"Saved preview overlay: {self.preview_png_path}")
        print(
            "Similarity: scale={:.6f}, rotation={:.3f} deg, translation=({:.3f}, {:.3f}) m, rmse={:.3f} m".format(
                scale,
                np.degrees(rotation_rad),
                tx,
                ty,
                rmse_m,
            )
        )

    def run(self) -> dict | None:
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(WINDOW_NAME, self._handle_click)

        while True:
            canvas = self._render_canvas()
            cv2.imshow(WINDOW_NAME, canvas)
            key = cv2.waitKey(20) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("u"):
                self._undo_last()
            elif key == ord("r"):
                self._reset()
            elif key == ord("s"):
                try:
                    self._solve_and_save()
                except Exception as exc:
                    print(f"[alignment] {exc}", file=sys.stderr)

        cv2.destroyWindow(WINDOW_NAME)
        return self.saved_result


def parse_args() -> argparse.Namespace:
    repo_root = Path(__file__).resolve().parents[4]
    parser = argparse.ArgumentParser(
        description="Manual similarity calibration from NYUSH testmap coordinates to LiDAR rm_frame."
    )
    parser.add_argument(
        "--pcd",
        default=str(repo_root / "mapping_ws" / "test.pcd"),
        help="Path to the LiDAR PCD map used by fusion.",
    )
    parser.add_argument(
        "--test-map",
        default="/home/nyu/Desktop/NYUSH_Robotics_RM_RadarStation/images/my_map(m).jpg",
        help="Path to the NYUSH runtime testmap image (the base image behind /nyush_map_image).",
    )
    parser.add_argument(
        "--field-width-m",
        type=float,
        default=DEFAULT_FIELD_WIDTH_M,
        help="Physical width represented by the testmap image in meters.",
    )
    parser.add_argument(
        "--field-height-m",
        type=float,
        default=DEFAULT_FIELD_HEIGHT_M,
        help="Physical height represented by the testmap image in meters.",
    )
    parser.add_argument(
        "--resolution-m-per-px",
        type=float,
        default=DEFAULT_RESOLUTION_M_PER_PX,
        help="LiDAR top-down raster resolution in meters per pixel.",
    )
    parser.add_argument(
        "--z-min-m",
        type=float,
        default=DEFAULT_Z_MIN_M,
        help="Minimum z included when rasterizing the LiDAR map.",
    )
    parser.add_argument(
        "--z-max-m",
        type=float,
        default=DEFAULT_Z_MAX_M,
        help="Maximum z included when rasterizing the LiDAR map.",
    )
    parser.add_argument(
        "--padding-m",
        type=float,
        default=DEFAULT_PADDING_M,
        help="Extra padding around the LiDAR top-down image bounds.",
    )
    parser.add_argument(
        "--output",
        default=str(repo_root / "config" / "local" / "testmap_to_rm_frame.yaml"),
        help="Output YAML path for the saved similarity transform.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    pcd_path = Path(args.pcd).expanduser().resolve()
    testmap_path = Path(args.test_map).expanduser().resolve()
    output_yaml_path = Path(args.output).expanduser().resolve()
    topdown_png_path = output_yaml_path.with_name(output_yaml_path.stem + "_topdown.png")
    preview_png_path = output_yaml_path.with_name(output_yaml_path.stem + "_preview.png")

    if not pcd_path.exists():
        raise SystemExit(f"PCD path does not exist: {pcd_path}")
    if not testmap_path.exists():
        raise SystemExit(f"Testmap image path does not exist: {testmap_path}")

    testmap_bgr = cv2.imread(str(testmap_path))
    if testmap_bgr is None:
        raise SystemExit(f"Failed to read testmap image: {testmap_path}")

    lidar_topdown_bgr, lidar_meta = render_lidar_topdown(
        pcd_path,
        resolution_m_per_px=float(args.resolution_m_per_px),
        z_min_m=float(args.z_min_m),
        z_max_m=float(args.z_max_m),
        padding_m=float(args.padding_m),
    )

    ui = ManualAlignmentUI(
        testmap_bgr=testmap_bgr,
        lidar_topdown_bgr=lidar_topdown_bgr,
        testmap_path=testmap_path,
        pcd_path=pcd_path,
        output_yaml_path=output_yaml_path,
        topdown_png_path=topdown_png_path,
        preview_png_path=preview_png_path,
        field_width_m=float(args.field_width_m),
        field_height_m=float(args.field_height_m),
        lidar_meta=lidar_meta,
    )
    ui.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
