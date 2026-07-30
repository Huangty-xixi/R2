#!/bin/bash
#
# uppc 上位机启动脚本
# 由 systemd 服务拉起，开机自启
#
set -o pipefail

ROS2_SETUP="/opt/ros/humble/setup.bash"
LIVOX_SETUP="/home/shijue2/lidar/ws_livox/install/setup.bash"
KFS_SETUP="/home/shijue2/桌面/kfs_ws/install/setup.bash"
WORKSPACE_SETUP="/home/shijue2/main_R2/main_R2/R2/UPPC/install/setup.bash"

export PATH="/usr/bin:/bin:/usr/local/bin:/opt/ros/humble/bin:$PATH"
export LD_LIBRARY_PATH=""

if [ ! -f "$ROS2_SETUP" ]; then
    echo "ERROR: ROS2 not found at $ROS2_SETUP"
    exit 1
fi
if [ ! -f "$LIVOX_SETUP" ]; then
    echo "ERROR: livox workspace not found at $LIVOX_SETUP"
    exit 1
fi
if [ ! -f "$WORKSPACE_SETUP" ]; then
    echo "ERROR: uppc workspace not built, run: colcon build --symlink-install"
    exit 1
fi

source "$ROS2_SETUP"
source "$LIVOX_SETUP"
source "$KFS_SETUP"
source "$WORKSPACE_SETUP"

echo "[$(date '+%Y-%m-%d %H:%M:%S')] uppc 启动"
exec ros2 launch vision_grasp_bringup bringup.launch.py
