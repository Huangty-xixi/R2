# -*- coding: utf-8 -*-
"""
状态机节点 (state_machine)
编排主工作流程：
  INIT     初始化（等待依赖就绪）
  IDLE     空闲：LiDAR 里程计由 serial_bridge 持续发送；等待下位机 [开摄像头] 信号
  TRACKING 追踪：使能摄像头检测，持续接收目标中心深度
  ARRIVED  到达：目标稳定进入可抓取范围 → 发送 [到达] 信号 → 关闭摄像头

说明：
- LiDAR 里程计不在状态机内管理，独立持续运行（满足"全程发送"需求）。
- 摄像头开/关通过调用 camera_detector 的 SetDetection 服务实现。
"""

from enum import Enum

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from vision_grasp_interfaces.msg import TargetDepth
from vision_grasp_interfaces.srv import SetDetection


class State(Enum):
    INIT = 0
    IDLE = 1
    TRACKING = 2
    ARRIVED = 3


class StateMachine(Node):
    def __init__(self) -> None:
        super().__init__("state_machine")

        self.declare_parameter("camera_cmd_topic", "/camera_command")
        self.declare_parameter("target_topic", "/target_depth")
        self.declare_parameter("arrival_topic", "/arrival_trigger")
        self.declare_parameter("set_detection_service", "/camera_detector/set_detection")
        # 连续 N 帧 in_range 确认到达（防抖消抖）
        self.declare_parameter("arrival_stable_frames", 5)
        # 到达后是否自动回到 IDLE 等待下一轮
        self.declare_parameter("auto_reset", True)

        self.stable_need = int(self.get_parameter("arrival_stable_frames").value)
        self.auto_reset = bool(self.get_parameter("auto_reset").value)

        self.state = State.INIT
        self._in_range_count = 0

        self.arrival_pub = self.create_publisher(
            Bool, self.get_parameter("arrival_topic").value, 10)

        self.create_subscription(
            Bool, self.get_parameter("camera_cmd_topic").value,
            self._on_camera_cmd, 10)
        self.create_subscription(
            TargetDepth, self.get_parameter("target_topic").value,
            self._on_target, 10)

        self.det_client = self.create_client(
            SetDetection, self.get_parameter("set_detection_service").value)

        self._goto(State.IDLE)
        self.get_logger().info("state_machine started -> IDLE (waiting for camera on signal)")

    # ------------------------------------------------------ 状态转换
    def _goto(self, new: State) -> None:
        if new != self.state:
            self.get_logger().info(f"State: {self.state.name} -> {new.name}")
        self.state = new

    def _set_detection(self, enable: bool) -> None:
        if not self.det_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("SetDetection service unavailable, cannot toggle camera")
            return
        req = SetDetection.Request()
        req.enable = enable
        future = self.det_client.call_async(req)
        future.add_done_callback(
            lambda f: self.get_logger().info(
                f"Camera detection toggle enable={enable}: {self._resp_str(f)}"))

    @staticmethod
    def _resp_str(future) -> str:
        try:
            r = future.result()
            return f"{r.success} {r.message}"
        except Exception as exc:  # noqa: BLE001
            return f"call exception: {exc}"

    # ------------------------------------------------------ 回调
    def _on_camera_cmd(self, msg: Bool) -> None:
        if msg.data:
            # 下位机发送开摄像头信号
            if self.state in (State.IDLE, State.ARRIVED):
                self._in_range_count = 0
                self._set_detection(True)
                self._goto(State.TRACKING)
        else:
            # 关摄像头信号
            self._set_detection(False)
            self._goto(State.IDLE)

    def _on_target(self, msg: TargetDepth) -> None:
        if self.state != State.TRACKING:
            return
        if not msg.valid:
            self._in_range_count = 0
            return

        if msg.in_range:
            self._in_range_count += 1
        else:
            self._in_range_count = 0

        self.get_logger().info(
            f"Target [{msg.class_name}] center depth={msg.center_depth:.3f}m "
            f"in_range={msg.in_range} ({self._in_range_count}/{self.stable_need})",
            throttle_duration_sec=0.5)

        if self._in_range_count >= self.stable_need:
            self._on_arrived()

    def _on_arrived(self) -> None:
        # 发送到达信号 → 关摄像头
        self.arrival_pub.publish(Bool(data=True))
        self.get_logger().info("Target stably within acceptable range -> publishing [arrived], disabling camera detection")
        self._set_detection(False)
        self._in_range_count = 0
        self._goto(State.ARRIVED)
        if self.auto_reset:
            self._goto(State.IDLE)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StateMachine()
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
