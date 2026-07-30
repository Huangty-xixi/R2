# -*- coding: utf-8 -*-
"""
视觉抓取系统启动文件（无相机版本 nocamera）  ——  基于 bringup v4

与 bringup.launch.py 的区别：
  - 不启动 realsense-ros    （D435i 驱动）
  - 不启动 camera_detector  （YOLOv8 分割检测）

启动内容：
  1) livox_ros_driver2  MID360 雷达驱动
  2) fast_lio           激光 SLAM 里程计，发布 /Odometry
  3) lidar_odom_relay   里程计转发（/Odometry → /odometry → serial_bridge）
  4) serial_bridge      串口桥
  5) state_machine      主流程状态机

Launch 参数：
  start_livox:=true|false       是否启动雷达驱动  （默认 true）
  start_fastlio:=true|false     是否启动 FAST-LIO2 （默认 true）

注意：启动前需 source 雷达工作空间：
  source /home/shijue2/lidar/ws_livox/install/setup.bash
  source ~/RC2026/install/setup.bash
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# --------------------------------------------------------------------------
# 路径定义（外部工作空间均使用绝对路径，避免 source 顺序问题）
# --------------------------------------------------------------------------

# livox_ros_driver2
LIVOX_LAUNCH = os.path.join(
    "/home/shijue2/lidar/ws_livox/install/livox_ros_driver2",
    "share", "livox_ros_driver2", "launch_ROS2",
    "msg_MID360_launch.py",
)

# FAST-LIO2（同一工作空间 ws_livox，包名 fast_lio）
# 发布 /Odometry (nav_msgs/Odometry)，由 odom_relay 转发给 serial_bridge
FASTLIO_LAUNCH = os.path.join(
    "/home/shijue2/lidar/ws_livox/install/fast_livo",
    "share", "fast_livo", "launch",
    "mapping_mid360.launch.py",
)


def generate_launch_description():
    bringup_share = get_package_share_directory("vision_grasp_bringup")
    cfg = os.path.join(bringup_share, "config")

    serial_params   = os.path.join(cfg, "serial_params.yaml")
    lidar_params    = os.path.join(cfg, "lidar_params.yaml")
    sm_params       = os.path.join(cfg, "state_machine_params.yaml")

    # ── Launch 参数声明 ──────────────────────────────────────────────────────
    start_livox     = LaunchConfiguration("start_livox")
    start_fastlio   = LaunchConfiguration("start_fastlio")

    declare_livox = DeclareLaunchArgument(
        "start_livox", default_value="true",
        description="是否启动 livox_ros_driver2 雷达驱动")
    declare_fastlio = DeclareLaunchArgument(
        "start_fastlio", default_value="true",
        description="是否启动 FAST-LIO2 激光 SLAM 里程计")

    # ── 1) livox 雷达驱动 ───────────────────────────────────────────────────
    livox = GroupAction(
        condition=IfCondition(start_livox),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(LIVOX_LAUNCH),
            ),
        ],
    )

    # ── 2) FAST-LIO2 ────────────────────────────────────────────────────────
    # 订阅 /livox/lidar 点云，发布 /Odometry
    # mapping.launch.py 默认使用其包内 config/mid360.yaml，无需额外参数
    # 若需自定义 config，可取消下方 launch_arguments 注释并填入路径
    FASTLIVO_CONFIG = "/home/shijue2/lidar/ws_livox/install/fast_livo/share/fast_livo/config/mid360.yaml"
    FASTLIVO_CAM_CONFIG = "/home/shijue2/lidar/ws_livox/install/fast_livo/share/fast_livo/config/camera_mid360.yaml"

    # 确保系统 libusb (/usr/lib/x86_64-linux-gnu) 优先于 /opt/MVS/lib 下的旧版
    # libusb，否则 PCL(libpcl_io) 会因 undefined symbol: libusb_set_option 崩溃。
    _sys_lib = "/usr/lib/x86_64-linux-gnu"
    _ld_path = _sys_lib + ":" + os.environ.get("LD_LIBRARY_PATH", "")

    fastlio = Node(
        condition=IfCondition(start_fastlio),
        package="fast_livo",
        executable="fastlivo_mapping",
        name="laserMapping",
        parameters=[FASTLIVO_CONFIG, FASTLIVO_CAM_CONFIG],
        output="log",           # ← 只输出到日志，不刷屏
        ros_arguments=['--log-level', 'WARN'],
        additional_env={"LD_LIBRARY_PATH": _ld_path},
    )

    # ── 3) 里程计转发（/Odometry → /odometry）──────────────────────────────
    odom_relay = Node(
        package="vision_grasp_lidar",
        executable="odom_relay_node",
        name="lidar_odom_relay",
        output="screen",
        parameters=[lidar_params],
    )

    # ── 4) 串口桥 ───────────────────────────────────────────────────────────
    serial_bridge = Node(
        package="vision_grasp_serial",
        executable="serial_bridge_node",
        name="serial_bridge",
        output="screen",
        parameters=[serial_params],
    )

    # ── 5) 状态机 ───────────────────────────────────────────────────────────
    state_machine = Node(
        package="vision_grasp_state_machine",
        executable="state_machine_node",
        name="state_machine",
        output="screen",
        parameters=[sm_params],
    )

    return LaunchDescription([
        declare_livox,
        declare_fastlio,
        LogInfo(msg="🚀 Launching as Normal ROS Node (no camera)"),
        livox,            # 1) 雷达驱动
        fastlio,          # 2) FAST-LIO2
        odom_relay,       # 3) 里程计转发
        serial_bridge,    # 4) 串口桥
        state_machine,    # 5) 状态机
    ])
