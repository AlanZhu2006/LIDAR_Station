#!/bin/bash
# 通过直链下载并安装 TensorRT 开发包（不依赖 apt 源）
# 适用于 CUDA 11.8，Ubuntu 22.04
# 注意：libnvinfer-dev 依赖 libnvinfer-headers-dev（头文件在独立包中），必须一起装
set -e
BASE="https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64"
VER="10.13.0.35-1+cuda11.8"
WORK="/tmp/tensorrt_install_$$"
mkdir -p "$WORK" && cd "$WORK"

echo "=== 下载 TensorRT 10.13 开发包 (CUDA 11.8) ==="
echo "  - libnvinfer10 (运行时)"
wget -q --show-progress "$BASE/libnvinfer10_${VER}_amd64.deb"
echo "  - libnvinfer-headers-dev (头文件，NvInfer.h 等)"
wget -q --show-progress "$BASE/libnvinfer-headers-dev_${VER}_amd64.deb"
echo "  - libnvinfer-dev (开发库，依赖上面)"
wget -q --show-progress "$BASE/libnvinfer-dev_${VER}_amd64.deb"
echo "  - libnvonnxparsers10 (ONNX 解析运行时)"
wget -q --show-progress "$BASE/libnvonnxparsers10_${VER}_amd64.deb"
echo "  - libnvonnxparsers-dev (NvOnnxParser.h 头文件)"
wget -q --show-progress "$BASE/libnvonnxparsers-dev_${VER}_amd64.deb"

echo "=== 安装（顺序：运行时 → 头文件 → 开发库）==="
sudo dpkg -i libnvinfer10_${VER}_amd64.deb \
             libnvinfer-headers-dev_${VER}_amd64.deb \
             libnvinfer-dev_${VER}_amd64.deb \
             libnvonnxparsers10_${VER}_amd64.deb \
             libnvonnxparsers-dev_${VER}_amd64.deb
sudo apt-get install -f -y

echo "=== 检查头文件 ==="
if ls /usr/include/x86_64-linux-gnu/NvInfer.h >/dev/null 2>&1 || ls /usr/include/NvInfer.h >/dev/null 2>&1; then
  echo "  NvInfer.h 已找到"
  find /usr -name 'NvInfer.h' 2>/dev/null | head -3
else
  echo "  未找到，请执行: find /usr -name NvInfer.h"
fi
cd - >/dev/null
rm -rf "$WORK"
echo "=== 完成。重新编译: cd ~/Desktop/RadarStation && source /opt/ros/humble/setup.bash && colcon build --packages-select tdt_vision ==="
