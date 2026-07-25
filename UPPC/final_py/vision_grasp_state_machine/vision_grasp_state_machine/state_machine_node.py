# -*- coding: utf-8 -*-
"""
状态机节点 (state_machine_node)

编排主工作流程——协调摄像头检测的启停：

   INIT     → 初始化，等依赖就绪
   IDLE     → 空闲状态：等 STM32 发来 [开摄像头] 信号
   TRACKING → 追踪模式：摄像头检测中，持续接收目标位置
   ARRIVED  → 到达：目标稳定在可抓取范围 → 发布到达信号 → 关摄像头 → 回到 IDLE

LiDAR 里程计不由本节点管理——serial_bridge 独立持续发送，不受状态机影响。
"""

from enum import Enum          # Python 枚举（= C 的 typedef enum）
                                # 比直接用 0/1/2/3 可读性强

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool                           # 标准消息：布尔
from vision_grasp_interfaces.msg import TargetDepth     # 自定义消息：检测结果
from vision_grasp_interfaces.srv import SetDetection    # 自定义服务：开关检测


class State(Enum):
    """状态机枚举（和 C 的 typedef enum 一样）"""
    INIT = 0       # 初始化
    IDLE = 1       # 空闲
    TRACKING = 2   # 追踪中
    ARRIVED = 3    # 已到达


class StateMachine(Node):
    def __init__(self) -> None:
        super().__init__("state_machine")    # 节点名

        # ── 可配参数 ──
        self.declare_parameter("camera_cmd_topic", "/camera_command")              # 订阅：摄像头开关指令
        self.declare_parameter("target_topic", "/target_depth")                   # 订阅：目标检测结果
        self.declare_parameter("arrival_topic", "/arrival_trigger")               # 发布：到达信号
        self.declare_parameter("set_detection_service", "/camera_detector/set_detection")  # 调用的服务
        self.declare_parameter("arrival_stable_frames", 5)                        # 连续多少帧 in_range 才判到达（防抖）
        self.declare_parameter("auto_reset", True)                                # 到达后是否自动回到 IDLE（等下一轮）

        self.stable_need = int(self.get_parameter("arrival_stable_frames").value)
        self.auto_reset = bool(self.get_parameter("auto_reset").value)

        # ── 状态 ──
        self.state = State.INIT                    # 初始状态 = INIT
        self._in_range_count = 0                   # "在可抓取范围内"的连续帧计数器

        # ── 发布者 ──
        self.arrival_pub = self.create_publisher(Bool, self.get_parameter("arrival_topic").value, 10)

        # ── 订阅者 ──
        self.create_subscription(Bool, self.get_parameter("camera_cmd_topic").value,
                                  self._on_camera_cmd, 10)  # 收 STM32 的摄像头开关指令
        self.create_subscription(TargetDepth, self.get_parameter("target_topic").value,
                                  self._on_target, 10)       # 收 camera_detector 的检测结果

        # ── 服务客户端（= C 里调另一个模块的函数） ──
        self.det_client = self.create_client(
            SetDetection, self.get_parameter("set_detection_service").value)  # 调 camera_detector 的开关服务

        # 启动 → 立刻跳到 IDLE
        self._goto(State.IDLE)
        self.get_logger().info("state_machine started -> IDLE (waiting for camera on signal)")

    # ═══════════════════════════════════════════════════════════ 状态转换辅助

    def _goto(self, new: State) -> None:
        """状态转换 + 打日志"""
        if new != self.state:                    # 只有真的变了才打日志（减少噪音）
            self.get_logger().info(f"State: {self.state.name} -> {new.name}")
            # {xxx.name} = Python 枚举的 .name 属性 = "INIT" / "IDLE" 等字符串
        self.state = new

    def _set_detection(self, enable: bool) -> None:
        """
        调 camera_detector 的 SetDetection 服务 → 开关检测
        wait_for_service(timeout_sec=2.0) → 等最多 2 秒（防服务还没启动的情况）
        call_async → 异步调用（不阻塞，结果通过回调接收）
        """
        if not self.det_client.wait_for_service(timeout_sec=2.0):  # 服务不可用
            self.get_logger().error("SetDetection service unavailable, cannot toggle camera")
            return
        req = SetDetection.Request()             # 创建请求对象
        req.enable = enable                      # True=开检测，False=关检测
        future = self.det_client.call_async(req) # 异步调用（不阻塞当前线程）
        # Python 的 lambda = 匿名函数（一行函数，不需要起名字）
        # 当服务调用完成时，ROS2 自动调这个 lambda → 打日志
        future.add_done_callback(
            lambda f: self.get_logger().info(
                f"Camera detection toggle enable={enable}: {self._resp_str(f)}"))

    @staticmethod                                # @staticmethod = 静态方法（不依赖 self，等价 C 的普通函数）
    def _resp_str(future) -> str:
        """把服务调用的 future 结果转成可读字符串（成功/失败）"""
        try:
            r = future.result()                  # 拿返回值（如果调用成功）
            return f"{r.success} {r.message}"
        except Exception as exc:                 # 捕获所有异常
            return f"call exception: {exc}"

    # ═══════════════════════════════════════════════════════════ ROS2 回调

    def _on_camera_cmd(self, msg: Bool) -> None:
        """
        收到 /camera_command 消息（= STM32 发来的摄像头开关指令）
        msg.data = True  → 开摄像头 → 进入 TRACKING 状态
        msg.data = False → 关摄像头 → 回到 IDLE
        """
        if msg.data:                             # True = 开摄像头
            # 只允许从 IDLE 或 ARRIVED 进入 TRACKING（防止在 TRACKING 时重复开）
            if self.state in (State.IDLE, State.ARRIVED):  # Python in 元组：等价于 state==IDLE or state==ARRIVED
                self._in_range_count = 0          # 重置到达计数器
                self._set_detection(True)          # 调服务开检测
                self._goto(State.TRACKING)
        else:                                    # False = 关摄像头
            self._set_detection(False)             # 关检测
            self._goto(State.IDLE)

    def _on_target(self, msg: TargetDepth) -> None:
        """
        收到 camera_detector 发来的检测结果
        只在 TRACKING 状态才处理（其他状态忽略）
        连续 stable_need 帧 in_range=True → 判定"到达" → 发布到达信号 → 关摄像头
        """
        if self.state != State.TRACKING: return   # 不在追踪状态 → 忽略

        if not msg.valid:                         # 无效帧（深度为空、置信度不够等）
            self._in_range_count = 0              # 重置计数
            return

        if msg.in_range:                          # 目标在可抓取范围内
            self._in_range_count += 1              # 累计连续帧数
        else:                                      # 不在范围内
            self._in_range_count = 0               # 重置计数

        # 打日志（throttle_duration_sec=0.5：每 0.5 秒最多打一次，防止刷屏）
        self.get_logger().info(
            f"Target [{msg.class_name}] center depth={msg.center_depth:.3f}m "
            f"in_range={msg.in_range} ({self._in_range_count}/{self.stable_need})",
            throttle_duration_sec=0.5)

        # 连续 N 帧都在范围内 → 判到达
        if self._in_range_count >= self.stable_need:
            self._on_arrived()

    def _on_arrived(self) -> None:
        """目标已到达 → 发布到达信号 → 关摄像头 → 可选自动复位"""
        self.arrival_pub.publish(Bool(data=True))    # 发布 /arrival_trigger = True
        self.get_logger().info("Target stably within acceptable range -> "
                               "publishing [arrived], disabling camera detection")
        self._set_detection(False)                    # 关摄像头检测
        self._in_range_count = 0                      # 重置计数器
        self._goto(State.ARRIVED)
        if self.auto_reset:                           # 如果配置了自动复位（默认 True）
            self._goto(State.IDLE)                    # 立刻回到 IDLE，等下一轮


def main(args=None) -> None:
    """ROS2 入口"""
    rclpy.init(args=args)
    node = StateMachine()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == "__main__":
    main()
