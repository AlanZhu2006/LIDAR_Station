#!/bin/bash

# 当接收到SIGINT信号时，结束脚本执行
trap "echo 'Script terminated by user'; exit" SIGINT
export time=$(date +%Y-%m-%d-%H-%M-%S)


# 获取脚本所在的绝对路径
SCRIPT_PATH=$(dirname $(realpath $0))
cd $SCRIPT_PATH/..
echo $SCRIPT_PATH

# 设置程序（50°倾斜用 lidar_pitch_deg:=-50）
PROGRAM="dynamic_cloud lidar.launch.py"
LAUNCH_ARGS="map_file:=mapping_ws/test.pcd ceiling_z_max:=100.0"

source ./install/setup.zsh
echo "to be launched"
while true
do
    if pgrep -f "lidar.launch.py" > /dev/null
    then
        echo "$PROGRAM is running~"
    else
        echo "$PROGRAM launch failed!"
        LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0 ros2 launch $PROGRAM $LAUNCH_ARGS
    fi

    # 等待一段时间再次检查
    sleep 1
done
