# -*- coding: utf-8 -*-
"""
视觉抓取系统总启动文件 (bringup)  ——  v4

启动内容：
  1) realsense-ros      D435i 驱动，开启对齐深度
  2) livox_ros_driver2  MID360 雷达驱动
  3) fast_lio           激光 SLAM 里程计，发布 /Odometry
  4) lidar_odom_relay   里程计转发（/Odometry → /odometry → serial_bridge）
  5) serial_bridge      串口桥
  6) camera_detector    YOLOv8 分割检测
  7) state_machine      主流程状态机

Launch 参数：
  start_realsense:=true|false   是否启动 D435i 驱动（默认 true）
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

REALSENSE_LAUNCH = os.path.join(
    get_package_share_directory("realsense2_camera"),
    "launch", "rs_launch.py",
)

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

    camera_params   = os.path.join(cfg, "camera_params.yaml")
    hand_eye_params = os.path.join(cfg, "hand_eye_calibration.yaml")
    serial_params   = os.path.join(cfg, "serial_params.yaml")
    lidar_params    = os.path.join(cfg, "lidar_params.yaml")
    sm_params       = os.path.join(cfg, "state_machine_params.yaml")

    # ── Launch 参数声明 ──────────────────────────────────────────────────────
    start_realsense = LaunchConfiguration("start_realsense")
    start_livox     = LaunchConfiguration("start_livox")
    start_fastlio   = LaunchConfiguration("start_fastlio")

    declare_realsense = DeclareLaunchArgument(
        "start_realsense", default_value="true",
        description="是否启动 RealSense D435i 驱动")
    declare_livox = DeclareLaunchArgument(
        "start_livox", default_value="true",
        description="是否启动 livox_ros_driver2 雷达驱动")
    declare_fastlio = DeclareLaunchArgument(
        "start_fastlio", default_value="true",
        description="是否启动 FAST-LIO2 激光 SLAM 里程计")

    # ── 1) RealSense D435i ──────────────────────────────────────────────────
    realsense = GroupAction(
        condition=IfCondition(start_realsense),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(REALSENSE_LAUNCH),
                launch_arguments={
                    "align_depth.enable": "true",
                    "pointcloud.enable":  "false",
                }.items(),
            ),
        ],
    )

    # ── 2) livox 雷达驱动 ───────────────────────────────────────────────────
    livox = GroupAction(
        condition=IfCondition(start_livox),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(LIVOX_LAUNCH),
            ),
        ],
    )

    # ── 3) FAST-LIO2 ────────────────────────────────────────────────────────
    # 订阅 /livox/lidar 点云，发布 /Odometry
    # mapping.launch.py 默认使用其包内 config/mid360.yaml，无需额外参数
    # 若需自定义 config，可取消下方 launch_arguments 注释并填入路径
    FASTLIVO_CONFIG = "/home/shijue2/lidar/ws_livox/install/fast_livo/share/fast_livo/config/mid360.yaml"
    FASTLIVO_CAM_CONFIG = "/home/shijue2/lidar/ws_livox/install/fast_livo/share/fast_livo/config/camera_mid360.yaml"

    fastlio = Node(
        condition=IfCondition(start_fastlio),
        package="fast_livo",
        executable="fastlivo_mapping",
        name="laserMapping",
        parameters=[FASTLIVO_CONFIG, FASTLIVO_CAM_CONFIG],
        output="log",           # ← 只输出到日志，不刷屏
        ros_arguments=['--log-level', 'WARN'],
    )

    # ── 4) 里程计转发（/Odometry → /odometry）──────────────────────────────
    odom_relay = Node(
        package="vision_grasp_lidar",
        executable="odom_relay_node",
        name="lidar_odom_relay",
        output="screen",
        parameters=[lidar_params],
    )

    # ── 5) 串口桥 ───────────────────────────────────────────────────────────
    serial_bridge = Node(
        package="vision_grasp_serial",
        executable="serial_bridge_node",
        name="serial_bridge",
        output="screen",
        parameters=[serial_params],
    )

    # ── 6) 摄像头检测节点 ───────────────────────────────────────────────────
    camera_detector = Node(
        package="vision_grasp_camera",
        executable="detector_node",
        name="camera_detector",
        output="screen",
        parameters=[camera_params, hand_eye_params],
    )

    # ── 7) 状态机 ───────────────────────────────────────────────────────────
    state_machine = Node(
        package="vision_grasp_state_machine",
        executable="state_machine_node",
        name="state_machine",
        output="screen",
        parameters=[sm_params],
    )

    return LaunchDescription([
        declare_realsense,
        declare_livox,
        declare_fastlio,
        LogInfo(msg="🚀 Launching as Normal ROS Node"),
        realsense,        # 1) D435i
        livox,            # 2) 雷达驱动
        fastlio,          # 3) FAST-LIO2
        odom_relay,       # 4) 里程计转发
        serial_bridge,    # 5) 串口桥
        camera_detector,  # 6) 摄像头检测
        state_machine,    # 7) 状态机
    ])