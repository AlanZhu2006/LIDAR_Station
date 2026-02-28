#!/bin/bash

# 当接收到SIGINT信号时，结束脚本执行
trap "echo 'Script terminated by user'; exit" SIGINT
current_time=$(date +"%Y-%m-%d_%H-%M-%S")

# 获取脚本所在的绝对路径
SCRIPT_PATH=$(dirname $(realpath $0))
cd $SCRIPT_PATH/..

# 设置程序路径
PROGRAM="tdt_vision calib_rosbag.launch.py"
ROSBAG_FILE=${ROSBAG_FILE:-}

source ./install/setup.zsh
if [ -z "$ROSBAG_FILE" ]; then
    echo "ROSBAG_FILE is empty. Example:"
    echo "export ROSBAG_FILE=/absolute/path/to/merged_bag_0.db3"
    exit 1
fi

while true
do
    # 检查程序是否正在运行
    if pgrep -f $PROGRAM > /dev/null
    then
        echo "$PROGRAM is running~"
    else
        echo "$PROGRAM launch failed!"

        # 启动程序
        ros2 launch $PROGRAM rosbag_file:=$ROSBAG_FILE
    fi

    # 等待一段时间再次检查
    sleep 1
done
