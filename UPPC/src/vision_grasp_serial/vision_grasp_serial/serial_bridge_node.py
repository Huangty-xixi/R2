# -*- coding: utf-8 -*-
"""
serial_bridge_node.py  ——  串口桥节点
唯一持有物理串口的 ROS 2 节点，所有其他节点通过话题与本节点交互。

上行（NUC → STM32）：
    /odometry        nav_msgs/Odometry
                     → CMD_ODOM(0x01)，全程持续按 send_rate_hz 发送
    /target_depth    vision_grasp_interfaces/TargetDepth
                     → CMD_KFS(0x03)        检测类别 + 相机坐标
                     → CMD_KFS_LATERAL_ERR(0x06)  相机坐标（下位机横向误差校正）
                     两帧同时发送，互补。valid=False 时丢弃。

下行（STM32 → NUC）：
    CMD_GO_ZONE_I(0x15)   → /camera_command True       ★ 开摄像头 ★
    CMD_STATUS(0x12) IDLE/DONE → /camera_command False ★ 关摄像头 ★
    CMD_STATUS 其他状态        → /robot_status UInt8   状态机使用
    CMD_ZONE_I_INFO(0x13)      → /zone_i_info  String  JSON 格式
    CMD_DOCK_OK(0x14)          → /dock_ok Bool
    CMD_ACK(0x10)              → 仅打日志
    CMD_RESET_ODOM(0x11)       → /odom_reset Bool(True) ★ 里程计归零 ★
    CMD_DEBUG_HEADING(0x20)    → /debug_heading String  JSON 格式
    CMD_DEBUG_NAV(0x21)        → /debug_nav     String  JSON 格式

CMD_RESET_ODOM (0x11)：
    协议中 0x11 为预留未定义字节，此处复用为里程计原点重置指令。
    下位机发送空载荷帧（A5 5A 11 00 00 11），上位机将当前位姿设为新原点。
    适用于比赛开始前快速归零，无需重启 SLAM 节点。
"""

import json
import threading
import struct

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, UInt8, String, Float32, Float32MultiArray

from vision_grasp_interfaces.msg import TargetDepth

from .protocol import (
    # 编码
    encode_odom,
    encode_kfs_single,
    encode_kfs_lateral_err,
    # 解码
    decode_ack,
    decode_zone_i_info,
    decode_debug_heading,
    decode_debug_nav,
    # 解析器
    FrameParser,
    # 命令字
    CMD_ACK,
    CMD_STATUS,
    CMD_ZONE_I_INFO,
    CMD_DOCK_OK,
    CMD_GO_ZONE_I,
    CMD_DEBUG_HEADING,
    CMD_DEBUG_NAV,
    # 状态枚举
    RC_STATE_IDLE,
    RC_STATE_DONE,
    # 坐标转换
    quat_to_euler_deg,
)

try:
    import serial
except ImportError:
    serial = None

# ── 本节点局部扩展命令字（不修改 protocol.py） ──────────────────────────────
# 0x11 在协议中位于 CMD_ACK(0x10) 和 CMD_STATUS(0x12) 之间，为预留未定义字节。
# 下位机发送空载荷帧（A5 5A 11 00 00 11）触发上位机里程计归零。
CMD_RESET_ODOM = 0x11
CMD_STOP_ZONE_I = 0x16 

class SerialBridge(Node):
    def __init__(self) -> None:
        super().__init__("serial_bridge")

        # ── 参数声明（由 serial_params.yaml 覆盖） ──────────────────────────
        self.declare_parameter("port",                "/dev/ttyUSB_online")
        self.declare_parameter("baudrate",            115200)
        self.declare_parameter("send_rate_hz",        50.0)    # ODOM 上行频率
        self.declare_parameter("read_poll_hz",        200.0)   # 串口读取轮询频率
        self.declare_parameter("odom_topic",          "/odometry")
        self.declare_parameter("target_topic",        "/target_depth")
        self.declare_parameter("camera_cmd_topic",    "/camera_command")
        self.declare_parameter("robot_status_topic",  "/robot_status")
        self.declare_parameter("dock_ok_topic",       "/dock_ok")
        self.declare_parameter("zone_i_info_topic",   "/zone_i_info")
        self.declare_parameter("debug_heading_topic", "/debug_heading")
        self.declare_parameter("debug_nav_topic",     "/debug_nav")
        self.declare_parameter("odom_reset_topic",    "/odom_reset")
        self.declare_parameter("chassis_cmd_topic", "/chassis_cmd")
        self.declare_parameter("kfs_cmd_topic",     "/kfs_cmd")
        self.declare_parameter("lift_cmd_topic",    "/lift_cmd")
        self.declare_parameter("flow_cmd_topic",    "/flow_cmd")
        self.declare_parameter("zone_cmd_topic",    "/zone_cmd")
        self.declare_parameter("pc_estop_topic",    "/pc_estop")
        self.declare_parameter("weapon_cmd_topic",  "/weapon_cmd")
        self.declare_parameter("kfs_pos_cmd_topic", "/kfs_pos_cmd")

        self.port      = self.get_parameter("port").value
        self.baud      = int(self.get_parameter("baudrate").value)
        self.send_rate = float(self.get_parameter("send_rate_hz").value)
        poll_hz        = float(self.get_parameter("read_poll_hz").value)

        # ── 串口 ─────────────────────────────────────────────────────────────
        self._ser      = None
        self._ser_lock = threading.Lock()
        self._open_serial()

        # 下行帧解析器（与下位机 rc_feed_byte 状态机对称）
        self._parser = FrameParser()

        # ── 发布者 ───────────────────────────────────────────────────────────
        self.cam_cmd_pub    = self.create_publisher(
            Bool,   self.get_parameter("camera_cmd_topic").value,    10)
        self.status_pub     = self.create_publisher(
            UInt8,  self.get_parameter("robot_status_topic").value,  10)
        self.dock_ok_pub    = self.create_publisher(
            Bool,   self.get_parameter("dock_ok_topic").value,       10)
        self.zone_info_pub  = self.create_publisher(
            String, self.get_parameter("zone_i_info_topic").value,   10)
        self.dbg_heading_pub = self.create_publisher(
            String, self.get_parameter("debug_heading_topic").value, 10)
        self.dbg_nav_pub    = self.create_publisher(
            String, self.get_parameter("debug_nav_topic").value,     10)
        # ★ 新增：里程计归零触发话题
        self.odom_reset_pub = self.create_publisher(
            Bool,   self.get_parameter("odom_reset_topic").value,    1)

        # ── 订阅者 ───────────────────────────────────────────────────────────
        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST)

        self._latest_odom: Odometry | None = None

        self.create_subscription(
            Odometry,
            self.get_parameter("odom_topic").value,
            self._on_odom, sensor_qos)

        self.create_subscription(
            TargetDepth,
            self.get_parameter("target_topic").value,
            self._on_target, 10)

        # ── PC 控制指令订阅（键盘遥控 → 话题 → 串口 → STM32） ──
        self.create_subscription(Float32MultiArray,
            self.get_parameter("chassis_cmd_topic").value,
            self._on_chassis_cmd, 10)
        self.create_subscription(Float32,
            self.get_parameter("kfs_cmd_topic").value,
            self._on_kfs_cmd, 10)
        self.create_subscription(Float32,
            self.get_parameter("lift_cmd_topic").value,
            self._on_lift_cmd, 10)
        self.create_subscription(Float32,
            self.get_parameter("flow_cmd_topic").value,
            self._on_flow_cmd, 10)
        self.create_subscription(Float32,
            self.get_parameter("zone_cmd_topic").value,
            self._on_zone_cmd, 10)
        self.create_subscription(Float32,
            self.get_parameter("pc_estop_topic").value,
            self._on_pc_estop, 10)
        self.create_subscription(Float32,
            self.get_parameter("weapon_cmd_topic").value,
            self._on_weapon_cmd, 10)
        self.create_subscription(Float32MultiArray,
            self.get_parameter("kfs_pos_cmd_topic").value,
            self._on_kfs_pos_cmd, 10)

        # ── 定时器 ───────────────────────────────────────────────────────────
        self.create_timer(1.0 / max(self.send_rate, 1.0), self._tick_odom)
        self.create_timer(1.0 / max(poll_hz, 1.0),        self._tick_read)

        self.get_logger().info(
            f"serial_bridge 启动  port={self.port}  baud={self.baud}  "
            f"odom_rate={self.send_rate} Hz")

    # ═══════════════════════════════════════════════════════════ 串口管理
    def _open_serial(self) -> None:
        if serial is None:
            self.get_logger().error("未安装 pyserial（pip install pyserial）")
            return
        try:
            self._ser = serial.Serial(self.port, self.baud, timeout=0)
            self.get_logger().info(f"串口已打开：{self.port} @ {self.baud}")
        except Exception as exc:                    # noqa: BLE001
            self._ser = None
            self.get_logger().error(f"串口打开失败：{exc}")

    def _write(self, frame: bytes) -> None:
        """向串口写入一帧，失败时标记串口无效等待下次重连。"""
        if self._ser is None:
            self._open_serial()
            if self._ser is None:
                return
        try:
            with self._ser_lock:
                self._ser.write(frame)
        except Exception as exc:                    # noqa: BLE001
            self.get_logger().warn(f"串口写入失败：{exc}")
            self._ser = None

    # ═══════════════════════════════════════════════════════════ 上行
    def _on_odom(self, msg: Odometry) -> None:
        """缓存最新里程计，由定时器限频发送，避免阻塞订阅回调。"""
        self._latest_odom = msg

    def _tick_odom(self) -> None:
        """定时器：将 FAST-LIO2 里程计打包为 CMD_ODOM(0x01) 帧上行。

        字段映射：
            p0 = position.x  → 下位机 handle_odom 读 data[0]  → robot_y（+偏移）
            p1 = position.y  → 下位机 handle_odom 读 data[4]  → ±robot_x（+偏移）
            z  = position.z  → 高度 (m)
            roll/pitch/yaw   → 由四元数转换为度，下位机执行 wrap_deg_180(yaw)
        坐标系映射和偏移补偿完全由下位机内部处理，上位机直接传原始值。
        """
        msg = self._latest_odom
        if msg is None:
            return
        pos  = msg.pose.pose.position
        q    = msg.pose.pose.orientation
        roll, pitch, yaw = quat_to_euler_deg(q.x, q.y, q.z, q.w)
        frame = encode_odom(pos.x, pos.y, pos.z, roll, pitch, yaw)
        self._write(frame)

    def _on_target(self, msg: TargetDepth) -> None:
        """收到摄像头检测结果，同时发送两个上行帧：

        1. CMD_KFS(0x03)：提供类别编号 + 相机坐标，供下位机抓取规划
           载荷：num=1 + [class_id + cam_x + cam_y + cam_z]
        2. CMD_KFS_LATERAL_ERR(0x06)：提供相机坐标，供下位机横向误差校正
           载荷：cam_x + cam_y + cam_z（z = center_depth）
        两帧携带相同坐标，各自对应下位机不同的处理流程。
        valid=False 时不发送（深度无效帧）。
        """
        if not msg.valid:
            return
        c = msg.position_camera   # 相机坐标系反投影结果

        # ① CMD_KFS：类别 + 相机坐标
        self._write(encode_kfs_single(msg.class_id, c.x, c.y, c.z))

        # ② CMD_KFS_LATERAL_ERR：相机坐标（下位机用于横向误差计算）
        self._write(encode_kfs_lateral_err(c.x, c.y, c.z))

    # ═══════════════════════════════════════════════════════════ PC 控制指令上行

    def _on_chassis_cmd(self, msg: Float32MultiArray) -> None:
        """PC→STM32 0x30：底盘速度 Vx,Vy,Vw（3×float32 LE）"""
        if len(msg.data) < 3:
            return
        payload = struct.pack('<fff', float(msg.data[0]), float(msg.data[1]), float(msg.data[2]))
        frame = bytes([0xA5, 0x5A, 0x30, 0x0C, 0x00]) + payload + bytes([0x00])
        self._write(frame)

    def _on_kfs_cmd(self, msg: Float32) -> None:
        """PC→STM32 0x32：KFS 取放动作（1字节：0=停 1=取 2=放）"""
        frame = bytes([0xA5, 0x5A, 0x32, 0x01, 0x00, int(msg.data) & 0xFF, 0x00])
        self._write(frame)

    def _on_lift_cmd(self, msg: Float32) -> None:
        """PC→STM32 0x33：抬升速度（1×float32 LE）"""
        payload = struct.pack('<f', float(msg.data))
        frame = bytes([0xA5, 0x5A, 0x33, 0x04, 0x00]) + payload + bytes([0x00])
        self._write(frame)

    def _on_flow_cmd(self, msg: Float32) -> None:
        """PC→STM32 0x34：流程触发（1字节）"""
        frame = bytes([0xA5, 0x5A, 0x34, 0x01, 0x00, int(msg.data) & 0xFF, 0x00])
        self._write(frame)

    def _on_zone_cmd(self, msg: Float32) -> None:
        """PC→STM32 0x35：业务zone启动（1字节）"""
        frame = bytes([0xA5, 0x5A, 0x35, 0x01, 0x00, int(msg.data) & 0xFF, 0x00])
        self._write(frame)

    def _on_pc_estop(self, msg: Float32) -> None:
        """PC→STM32 0x31：急停（0字节payload）"""
        frame = bytes([0xA5, 0x5A, 0x31, 0x00, 0x00, 0x00])
        self._write(frame)

    def _on_weapon_cmd(self, msg: Float32) -> None:
        """PC→STM32 0x36：武器toggle（1字节：设备号）"""
        frame = bytes([0xA5, 0x5A, 0x36, 0x01, 0x00, int(msg.data) & 0xFF, 0x00])
        self._write(frame)

    def _on_kfs_pos_cmd(self, msg: Float32MultiArray) -> None:
        """PC→STM32 0x37~0x3B：KFS档位调节 [设备号, 方向]"""
        if len(msg.data) < 2:
            return
        dev = int(msg.data[0]) & 0xFF
        dir_val = int(msg.data[1]) & 0xFF
        cmd = 0x36 + dev
        frame = bytes([0xA5, 0x5A, cmd, 0x01, 0x00, dir_val, 0x00])
        self._write(frame)

    # ═══════════════════════════════════════════════════════════ 下行
    def _tick_read(self) -> None:
        """轮询串口，将字节流喂给 FrameParser 解析下行帧。"""
        if self._ser is None:
            return
        try:
            with self._ser_lock:
                n    = self._ser.in_waiting
                data = self._ser.read(n) if n else b""
        except Exception as exc:                    # noqa: BLE001
            self.get_logger().warn(f"串口读取失败：{exc}")
            self._ser = None
            return
        for frame in self._parser.feed(data):
            self._dispatch(frame)

    def _dispatch(self, frame) -> None:
        """分发解析完成的下行帧到对应 ROS 话题。"""
        cmd = frame.cmd

        if cmd == CMD_GO_ZONE_I:
            # 0x15：下位机请求进入 I 区 → 开摄像头
            self.cam_cmd_pub.publish(Bool(data=True))
            self.get_logger().info("收到 CMD_GO_ZONE_I(0x15) → 开摄像头")

        elif cmd == CMD_STOP_ZONE_I:                          # ← 加这一段
            # 0x16：下位机请求停止检测 → 关摄像头
            self.cam_cmd_pub.publish(Bool(data=False))
            self.get_logger().info("收到 CMD_STOP_ZONE_I(0x16) → 关摄像头")
        
        elif cmd == CMD_RESET_ODOM:
            # 0x11：预留字节复用为里程计归零指令
            # 下位机发送帧：A5 5A 11 00 00 11（空载荷）
            # 上位机发布 /odom_reset → odom_relay 记录当前位姿为新原点
            self.odom_reset_pub.publish(Bool(data=True))
            self.get_logger().info(
                "收到 CMD_RESET_ODOM(0x11) → 触发里程计归零，"
                "当前位姿将作为新原点")

        elif cmd == CMD_STATUS:
            if len(frame.payload) < 1:
                return
            state = frame.payload[0]
            self.status_pub.publish(UInt8(data=int(state)))
            self.get_logger().debug(f"收到 CMD_STATUS(0x12) state={state}")
            # IDLE(0) 或 DONE(4) → 任务结束，关摄像头
            if state in (RC_STATE_IDLE, RC_STATE_DONE):
                self.cam_cmd_pub.publish(Bool(data=False))
                self.get_logger().info(
                    f"CMD_STATUS state={state}（{'IDLE' if state == RC_STATE_IDLE else 'DONE'}）→ 关摄像头")

        elif cmd == CMD_DOCK_OK:
            # 0x14：R1 对接成功
            self.dock_ok_pub.publish(Bool(data=True))
            self.get_logger().info("收到 CMD_DOCK_OK(0x14)：R1 对接成功")

        elif cmd == CMD_ZONE_I_INFO:
            # 0x13：I 区 KFS 布局信息
            info = decode_zone_i_info(frame.payload)
            if info is not None:
                payload_json = json.dumps({
                    "detections": [
                        {"block_id": d.block_id, "kfs_type": d.kfs_type}
                        for d in info.detections
                    ]
                })
                self.zone_info_pub.publish(String(data=payload_json))
                self.get_logger().info(f"收到 CMD_ZONE_I_INFO(0x13)：{payload_json}")

        elif cmd == CMD_ACK:
            # 0x10：确认帧，仅打日志
            ack = decode_ack(frame.payload)
            if ack is not None:
                self.get_logger().info(
                    f"收到 CMD_ACK(0x10)：acked_cmd=0x{ack.acked_cmd:02X} "
                    f"code={'OK' if ack.code == 0 else 'ERR'}")

        elif cmd == CMD_DEBUG_HEADING:
            # 0x20：航向保持 PID 调试数据
            dbg = decode_debug_heading(frame.payload)
            if dbg is not None:
                msg = String(data=json.dumps({
                    "yaw_ref_deg":   round(dbg.yaw_ref_deg, 3),
                    "yaw_deg":       round(dbg.yaw_deg, 3),
                    "err_deg":       round(dbg.err_deg, 3),
                    "i_term":        round(dbg.i_term, 4),
                    "output":        round(dbg.output, 4),
                    "yaw_rate_dps":  round(dbg.yaw_rate_dps, 3),
                }))
                self.dbg_heading_pub.publish(msg)

        elif cmd == CMD_DEBUG_NAV:
            # 0x21：导航到点调试数据
            dbg = decode_debug_nav(frame.payload)
            if dbg is not None:
                msg = String(data=json.dumps({
                    "ex":      round(dbg.ex, 4),
                    "ey":      round(dbg.ey, 4),
                    "dist":    round(dbg.dist, 4),
                    "zone":    dbg.zone,
                    "vy_fwd":  round(dbg.vy_fwd, 4),
                    "vw_str":  round(dbg.vw_str, 4),
                }))
                self.dbg_nav_pub.publish(msg)

        else:
            self.get_logger().debug(f"忽略未处理下行 CMD 0x{cmd:02X}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

