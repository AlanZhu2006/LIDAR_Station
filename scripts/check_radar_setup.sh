#!/bin/bash
# =============================================================================
# RadarStation 雷达站 - Livox 连接完整检查脚本
# =============================================================================

set -e
cd "$(dirname "$0")/.."
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

LIVOX_IF="${LIVOX_IF:-enx00e04c2536b0}"
CFG="$PWD/src/livox_ros_driver2/config/MID360_config.json"

read_config_value() {
    local key="$1"
    if [ ! -f "$CFG" ]; then
        return 1
    fi
    python3 - "$CFG" "$key" <<'PY'
import json
import sys

cfg_path, key = sys.argv[1:3]
with open(cfg_path, "r", encoding="utf-8") as f:
    data = json.load(f)

if key == "lidar_ip":
    print(data["lidar_configs"][0]["ip"])
elif key == "host_ip":
    print(data["MID360"]["host_net_info"]["cmd_data_ip"])
else:
    raise SystemExit(1)
PY
}

DEFAULT_RADAR_IP="$(read_config_value lidar_ip 2>/dev/null || true)"
DEFAULT_HOST_IP="$(read_config_value host_ip 2>/dev/null || true)"
RADAR_IP="${LIVOX_LIDAR_IP:-${DEFAULT_RADAR_IP:-192.168.1.3}}"
HOST_IP="${LIVOX_HOST_IP:-${DEFAULT_HOST_IP:-192.168.1.5}}"

echo "=============================================="
echo "  Livox 雷达连接完整检查"
echo "=============================================="

# 1. 网口存在
echo ""
echo "[1/7] 检查网口 $LIVOX_IF ..."
if ip link show "$LIVOX_IF" &>/dev/null; then
    echo -e "  ${GREEN}✓${NC} 网口存在"
else
    echo -e "  ${RED}✗${NC} 网口不存在，请检查 USB 转网口是否插入"
    echo "  可用网口: $(ip link show | grep -E '^[0-9]+:' | awk -F: '{print $2}' | tr -d ' ')"
    exit 1
fi

# 2. 配置 IP
echo ""
echo "[2/7] 配置网口 IP $HOST_IP/24 ..."
sudo ip addr add $HOST_IP/24 dev "$LIVOX_IF" 2>/dev/null || true
if ip addr show "$LIVOX_IF" | grep -q "$HOST_IP"; then
    echo -e "  ${GREEN}✓${NC} IP 已配置"
else
    echo -e "  ${RED}✗${NC} IP 配置失败"
    exit 1
fi

# 3. ping 雷达
echo ""
echo "[3/7] 测试雷达连接 (ping $RADAR_IP) ..."
if ping -c 2 -W 2 $RADAR_IP &>/dev/null; then
    echo -e "  ${GREEN}✓${NC} 雷达可达"
else
    echo -e "  ${YELLOW}⚠${NC} 雷达不可达，请检查："
    echo "    - 雷达是否开机（指示灯亮）"
    echo "    - 网线是否接对（雷达 → USB转网口 → 电脑）"
    echo "    - 雷达 IP 可能不是 114，Mid-360 默认 IP=192.168.1.1XX（XX=序列号后两位）"
    echo "    - 可用 Livox Viewer2 扫描设备确认 IP"
fi

# 4. MID360 配置
echo ""
echo "[4/7] 检查 MID360_config.json ..."
if [ -f "$CFG" ]; then
    LIDAR_IP="$(read_config_value lidar_ip)"
    CONFIG_HOST_IP="$(read_config_value host_ip)"
    echo -e "  ${GREEN}✓${NC} 配置文件存在，雷达 IP 配置: $LIDAR_IP，主机 IP 配置: $CONFIG_HOST_IP"
else
    echo -e "  ${RED}✗${NC} 配置文件不存在"
fi

# 5. xfer_format（检测用 PointCloud2）
echo ""
echo "[5/7] 检查 Livox 驱动格式 ..."
if grep -q 'xfer_format.*=.*0' src/livox_ros_driver2/launch_ROS2/msg_MID360_launch.py 2>/dev/null; then
    echo -e "  ${GREEN}✓${NC} 默认 PointCloud2（检测/融合用）"
else
    echo -e "  ${YELLOW}⚠${NC} 请确认 msg_MID360_launch.py 中 xfer_format=0（检测用）"
fi

# 6. 地图文件
echo ""
echo "[6/7] 检查地图文件 ..."
MAP1="$PWD/mapping_ws/test.pcd"
MAP2="$PWD/config/RM2024.pcd"
if [ -f "$MAP1" ]; then
    echo -e "  ${GREEN}✓${NC} mapping_ws/test.pcd 存在 ($(ls -lh "$MAP1" | awk '{print $5}'))"
elif [ -f "$MAP2" ]; then
    echo -e "  ${GREEN}✓${NC} config/RM2024.pcd 存在"
else
    echo -e "  ${RED}✗${NC} 地图文件不存在，请先建图或复制地图"
fi

# 7. 编译
echo ""
echo "[7/7] 检查编译 ..."
if [ -f "install/livox_ros_driver2/lib/livox_ros_driver2/livox_ros_driver2_node" ] || [ -f "install/livox_ros_driver2/lib/livox_ros_driver2/liblivox_ros_driver2.so" ]; then
    echo -e "  ${GREEN}✓${NC} livox_ros_driver2 已编译"
else
    echo -e "  ${YELLOW}⚠${NC} 请运行: cd src/livox_ros_driver2 && ./build.sh humble"
fi

echo ""
echo "=============================================="
echo "  检查完成"
echo "=============================================="
echo ""
echo "若出现 bind failed，先执行："
echo "  sudo ip addr add 192.168.1.5/24 dev $LIVOX_IF"
echo "  pkill -f livox_ros_driver2_node   # 清理残留进程"
echo ""
echo "若 ping 不通，可尝试："
echo "  export LIVOX_IF=enx00e04c2536b0   # 或你的 USB 网口名"
echo "  ./scripts/check_radar_setup.sh"
echo ""
echo "启动融合: ./scripts/start_fusion.sh"
echo ""
