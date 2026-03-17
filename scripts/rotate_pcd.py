#!/usr/bin/env python3
"""
倾斜安装雷达建图后旋转 PCD 点云。支持 ASCII 和 binary。
支持 pitch/roll/yaw 组合，可修正「斜着斜」为「正着斜」。

用法:
  python3 scripts/rotate_pcd.py [input.pcd] [output.pcd] [--pitch -50]
  python3 scripts/rotate_pcd.py input.pcd out.pcd --pitch -50 --roll 2 --yaw 0
  默认：mapping_ws/test.pcd -> mapping_ws/test_rotated.pcd
"""
import argparse
import sys
from pathlib import Path

import numpy as np

try:
    import open3d as o3d
    HAS_OPEN3D = True
except ImportError:
    HAS_OPEN3D = False


def rotation_matrix_euler(roll_deg: float, pitch_deg: float, yaw_deg: float):
    """欧拉角旋转矩阵，顺序 ZYX（yaw-pitch-roll）。单位度。"""
    r = np.radians(roll_deg)
    p = np.radians(pitch_deg)
    y = np.radians(yaw_deg)
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def main():
    parser = argparse.ArgumentParser(description="旋转 PCD 点云（倾斜安装修正，支持 pitch/roll/yaw）")
    parser.add_argument("input", nargs="?", default="mapping_ws/test.pcd")
    parser.add_argument("output", nargs="?", default="mapping_ws/test_rotated.pcd")
    parser.add_argument("--pitch", type=float, default=-50.0,
                        help="绕Y轴(度)。雷达向前倾50°时用 -50 使地图水平")
    parser.add_argument("--roll", type=float, default=0.0,
                        help="绕X轴(度)。修正左右倾斜/斜着斜")
    parser.add_argument("--yaw", type=float, default=0.0,
                        help="绕Z轴(度)。修正水平旋转")
    args = parser.parse_args()

    base = Path(__file__).resolve().parent.parent
    input_path = base / args.input if not Path(args.input).is_absolute() else Path(args.input)
    output_path = base / args.output if not Path(args.output).is_absolute() else Path(args.output)

    if not input_path.exists():
        print(f"错误: 输入文件不存在 {input_path}")
        sys.exit(1)

    if HAS_OPEN3D:
        pcd = o3d.io.read_point_cloud(str(input_path))
        if len(pcd.points) == 0:
            print("错误: 点云为空")
            sys.exit(1)
        R = rotation_matrix_euler(args.roll, args.pitch, args.yaw)
        pcd.rotate(R, center=(0, 0, 0))
        output_path.parent.mkdir(parents=True, exist_ok=True)
        o3d.io.write_point_cloud(str(output_path), pcd)
    else:
        try:
            with open(input_path, "rb") as f:
                raw = f.read(500)
            if b"DATA binary" in raw or b"DATA binary_compressed" in raw:
                print("错误: PCD 为 binary 格式，需安装 open3d: pip3 install open3d")
                sys.exit(1)
            with open(input_path, encoding="utf-8") as f:
                lines = f.readlines()
            data_start = next(i + 1 for i, L in enumerate(lines) if L.strip() == "DATA ascii")
            pts = np.array([[float(x) for x in L.split()[:3]] for L in lines[data_start:] if len(L.split()) >= 3])
            R = rotation_matrix_euler(args.roll, args.pitch, args.yaw)
            pts = (R @ pts.T).T
            output_path.parent.mkdir(parents=True, exist_ok=True)
            with open(output_path, "w") as f:
                f.writelines(lines[:data_start])
                for p in pts:
                    f.write(f"{p[0]} {p[1]} {p[2]}\n")
        except (UnicodeDecodeError, StopIteration) as e:
            print(f"读取失败: {e}")
            print("FAST-LIO 保存 binary PCD，请安装: pip3 install open3d")
            sys.exit(1)

    rpy = f"roll={args.roll} pitch={args.pitch} yaw={args.yaw}"
    print(f"已保存: {output_path} ({rpy})")


if __name__ == "__main__":
    main()
