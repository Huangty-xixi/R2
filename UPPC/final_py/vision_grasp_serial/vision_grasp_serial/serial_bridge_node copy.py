# -*- coding: utf-8 -*-
"""
串口桥节点 (serial_bridge)  ——  适配协议 v3
唯一持有物理串口的节点，其余节点通过 ROS 话题与本节点交互。

上行 (上位机 NUC -> 下位机)：
    /odometry       (nav_msgs/Odometry)                   -> CMD_ODOM，按 send_rate_hz 限频持续上行
    /target_depth   (vision_grasp_interfaces/TargetDepth) -> CMD_DEPTH，含目标中心深度与 in_range

下行 (下位机 -> 上位机 NUC)：
    串口字节流解析完整帧（同上行帧结构，带 CRC）：
        CMD_CAM_ON  (0x10) -> 发布 /camera_command True  (开摄像头)
        CMD_CAM_OFF (0x11) -> 发布 /camera_command False (关摄像头)
    使用 FrameParser 解析，具备 CRC 校验 + 坏帧重同步能力，抗串口干扰。

注：到达信号并入 CMD_DEPTH 帧的 in_range 位，无需单独上行帧。
"""

import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from vision_grasp_interfaces.msg import TargetDepth

from .protocol import (
    FrameParser,   # 下行解析（CMD_CAM_ON / CMD_CAM_OFF），与上行帧格式相同
    encode_odom,
    encode_depth,
    CMD_CAM_ON,
    CMD_CAM_OFF,
)

try:
    import serial  # pyserial
except ImportError:  # 允许在无 pyserial 的仿真环境下仅做语法导入
    serial = None


def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """四元数 (x,y,z,w) 转航向角 yaw (rad)。"""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class SerialBridge(Node):
    def __init__(self) -> None:
        super().__init__("serial_bridge")

        # ---- 参数（可被 yaml 覆盖，见 serial_params.yaml）----
        self.declare_parameter("port",             "/dev/ttyUSB0")
        self.declare_parameter("baudrate",         115200)
        self.declare_parameter("send_rate_hz",     50.0)    # 里程计上行限频
        self.declare_parameter("read_poll_hz",     200.0)   # 串口读取轮询频率
        self.declare_parameter("odom_topic",       "/odometry")
        self.declare_parameter("target_topic",     "/target_depth")
        self.declare_parameter("camera_cmd_topic", "/camera_command")

        self.port      = self.get_parameter("port").value
        self.baud      = int(self.get_parameter("baudrate").value)
        self.send_rate = float(self.get_parameter("send_rate_hz").value)
        read_poll_hz   = float(self.get_parameter("read_poll_hz").value)

        # ---- 串口 ----
        self._ser      = None
        self._ser_lock = threading.Lock()
        self._open_serial()

        # 下行帧解析器（FrameParser，与上行完全相同的帧结构，带 CRC 校验）
        self._downlink_parser = FrameParser()

        # ---- 发布者：解析到的摄像头开关命令 ----
        self.cam_cmd_pub = self.create_publisher(
            Bool, self.get_parameter("camera_cmd_topic").value, 10)

        # ---- 订阅者 ----
        sensor_qos = QoSProfile(depth=10,
                                reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST)
        self._latest_odom = None

        self.create_subscription(
            Odometry,
            self.get_parameter("odom_topic").value,
            self._on_odom,
            sensor_qos)

        self.create_subscription(
            TargetDepth,
            self.get_parameter("target_topic").value,
            self._on_target,
            10)

        # ---- 定时器：限频上行里程计 ----
        odom_period = 1.0 / self.send_rate if self.send_rate > 0 else 0.02
        self.create_timer(odom_period, self._tick_send_odom)

        # ---- 定时器：轮询读取串口下行数据 ----
        read_period = 1.0 / max(read_poll_hz, 1.0)
        self.create_timer(read_period, self._tick_read)

        self.get_logger().info(
            f"serial_bridge 已启动  port={self.port}  baud={self.baud}  "
            f"odom_rate={self.send_rate} Hz")

    # ----------------------------------------------------------------- 串口
    def _open_serial(self) -> None:
        """尝试打开串口，失败时记录错误并在后续写/读时重试。"""
        if serial is None:
            self.get_logger().error("未安装 pyserial (pip install pyserial)")
            return
        try:
            self._ser = serial.Serial(self.port, self.baud, timeout=0)
            self.get_logger().info(f"已打开串口 {self.port}")
        except Exception as exc:          # noqa: BLE001
            self._ser = None
            self.get_logger().error(f"打开串口失败: {exc}，将在运行中重试")

    def _write(self, frame: bytes) -> None:
        """向串口写入一帧，失败时标记串口无效并等待重连。"""
        if self._ser is None:
            self._open_serial()
            if self._ser is None:
                return
        try:
            with self._ser_lock:
                self._ser.write(frame)
        except Exception as exc:          # noqa: BLE001
            self.get_logger().warn(f"串口写入失败: {exc}")
            self._ser = None

    # ------------------------------------------------------------ 上行回调
    def _on_odom(self, msg: Odometry) -> None:
        """缓存最新里程计，由定时器限频发送，避免抢占串口带宽。"""
        self._latest_odom = msg

    def _tick_send_odom(self) -> None:
        """定时器回调：按 send_rate_hz 将里程计打帧上行。"""
        msg = self._latest_odom
        if msg is None:
            return
        p   = msg.pose.pose.position
        q   = msg.pose.pose.orientation
        v   = msg.twist.twist
        yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
        self._write(encode_odom(p.x, p.y, yaw,
                                v.linear.x, v.linear.y, v.angular.z))

    def _on_target(self, msg: TargetDepth) -> None:
        """收到目标中心深度消息即打 CMD_DEPTH 帧上行。
        valid=False 时深度无效，丢弃本帧不上行。
        in_range 位供下位机判断目标是否进入合理夹取范围（到达判断）。
        """
        if not msg.valid:
            return
        g = msg.position_gripper
        self._write(encode_depth(
            msg.class_id, msg.center_depth,
            g.x, g.y, g.z,
            msg.in_range))

    # ------------------------------------------------------------ 下行读取
    def _tick_read(self) -> None:
        """定时器回调：轮询串口缓冲，用 FrameParser 解析下行帧（带 CRC）。

        下行帧格式与上行完全相同（AA 55 CMD LEN PAYLOAD CRC16），
        FrameParser 自动丢弃 CRC 错误帧，防止干扰导致误触发。
        """
        if self._ser is None:
            return
        try:
            with self._ser_lock:
                n    = self._ser.in_waiting
                data = self._ser.read(n) if n else b""
        except Exception as exc:          # noqa: BLE001
            self.get_logger().warn(f"串口读取失败: {exc}")
            self._ser = None
            return
        if not data:
            return

        for frame in self._downlink_parser.feed(data):
            if frame.cmd == CMD_CAM_ON:
                self.cam_cmd_pub.publish(Bool(data=True))
                self.get_logger().info("收到下位机【开摄像头】CMD_CAM_ON (0x10)，CRC 校验通过")
            elif frame.cmd == CMD_CAM_OFF:
                self.cam_cmd_pub.publish(Bool(data=False))
                self.get_logger().info("收到下位机【关摄像头】CMD_CAM_OFF (0x11)，CRC 校验通过")
            # 其他 CMD 暂不处理，协议扩展时在此添加


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