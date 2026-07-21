#!/usr/bin/env python3
"""
qrcode_control.py（扳机 + 自定义信息 + 支持序列动作）
- 订阅 /qrcode/text (String)：二维码内容（任意字符串）
- 订阅 /qr_fire (Bool)：扳机。只有 fire=True 后，才消费一次二维码内容
- 发布 /qr/custom_info (String)：把二维码内容原样发布出去
- 支持动作序列：用 ';' 分隔多个动作
  每个动作格式：
    MOVE:FORWARD:0.5
    MOVE:BACK:0.5
    TURN:LEFT:1.57
    TURN:RIGHT:1.57
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist


class QRCodeControl(Node):
    def __init__(self):
        super().__init__('qrcode_control')

        self.declare_parameter('input_topic', '/qrcode/text')
        self.declare_parameter('fire_topic', '/qr_fire')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')          # 3D里用 /cmd_vel
        self.declare_parameter('info_topic', '/qr/custom_info')

        # 速度（距离换算成时间已写在 txt 里，这里只决定实际速度）
        self.declare_parameter('linear_speed', 0.2)   # m/s
        self.declare_parameter('angular_speed', 1.0)  # rad/s

        self.input_topic = str(self.get_parameter('input_topic').value)
        self.fire_topic = str(self.get_parameter('fire_topic').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.info_topic = str(self.get_parameter('info_topic').value)

        self.pub_cmd = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.pub_info = self.create_publisher(String, self.info_topic, 10)

        self.sub_text = self.create_subscription(String, self.input_topic, self.on_text, 10)
        self.sub_fire = self.create_subscription(Bool, self.fire_topic, self.on_fire, 10)

        # 扳机
        self._armed = False

        # 序列执行状态
        self._queue = []          # list of (lin_x, ang_z, duration)
        self._step_timer = None   # one-shot timer

        self.get_logger().info(f"subscribe text: {self.input_topic}")
        self.get_logger().info(f"subscribe fire: {self.fire_topic}")
        self.get_logger().info(f"publish info: {self.info_topic}")
        self.get_logger().info(f"publish cmd:  {self.cmd_vel_topic}")
        self.get_logger().info("Mode: ONLY execute after /qr_fire=True (one-shot)")
        self.get_logger().info("Sequence format: ACTION;ACTION;...  e.g. MOVE:FORWARD:0.5;TURN:LEFT:1.57;MOVE:FORWARD:0.5")

    def on_fire(self, msg: Bool):
        if msg.data:
            self._armed = True
            self.get_logger().info("Armed: next QR text will be consumed once")

    def publish_twist(self, lin_x: float, ang_z: float):
        t = Twist()
        t.linear.x = float(lin_x)
        t.angular.z = float(ang_z)
        self.pub_cmd.publish(t)

    def stop(self):
        self.publish_twist(0.0, 0.0)

    def cancel_timer(self):
        if self._step_timer is not None:
            try:
                self._step_timer.cancel()
            except Exception:
                pass
            self._step_timer = None

    def parse_one_action(self, s: str):
        """
        输入：'MOVE:FORWARD:0.5'
        输出： (lin_x, ang_z, duration, ok)
        """
        parts = [p.strip() for p in s.split(':')]
        if len(parts) < 3:
            return 0.0, 0.0, 0.0, False

        kind = parts[0].upper()
        dire = parts[1].upper()
        try:
            dur = float(parts[2])
        except Exception:
            return 0.0, 0.0, 0.0, False

        lin = float(self.get_parameter('linear_speed').value)
        ang = float(self.get_parameter('angular_speed').value)

        if kind == "MOVE":
            if dire in ("FORWARD", "FRONT"):
                return lin, 0.0, dur, True
            if dire in ("BACK", "BACKWARD"):
                return -lin, 0.0, dur, True
            return 0.0, 0.0, 0.0, False

        if kind == "TURN":
            if dire == "LEFT":
                return 0.0, ang, dur, True
            if dire == "RIGHT":
                return 0.0, -ang, dur, True
            return 0.0, 0.0, 0.0, False

        return 0.0, 0.0, 0.0, False

    def parse_sequence(self, text: str):
        """
        支持 ';' 分隔多个动作
        返回 queue: [(lin, ang, dur), ...]
        """
        actions = [a.strip() for a in text.split(';') if a.strip()]
        queue = []
        for a in actions:
            lin_x, ang_z, dur, ok = self.parse_one_action(a)
            if not ok:
                return []
            queue.append((lin_x, ang_z, dur))
        return queue

    def run_next_step(self):
        # 队列空：结束
        if not self._queue:
            self.stop()
            self.get_logger().info("Sequence done. Stop.")
            return

        lin_x, ang_z, dur = self._queue.pop(0)

        # 执行本步
        self.publish_twist(lin_x, ang_z)
        self.get_logger().info(f"Step: lin={lin_x} ang={ang_z} dur={dur}s")

        # 定时结束本步，然后执行下一步
        self.cancel_timer()

        def _cb():
            self.stop()
            self.cancel_timer()
            # 小停顿也行（这里不加停顿，直接下一步）
            self.run_next_step()

        self._step_timer = self.create_timer(float(dur), _cb)

    def on_text(self, msg: String):
        text = (msg.data or "").strip()
        if not text:
            return

        # 没扳机：不消费（开机不会动，也不会重复触发）
        if not self._armed:
            return

        # 只消费一次：先关扳机
        self._armed = False

        # 永远发布自定义信息
        out = String()
        out.data = text
        self.pub_info.publish(out)
        self.get_logger().info(f"Custom info published: {text}")

        # 解析动作序列
        queue = self.parse_sequence(text)
        if not queue:
            self.get_logger().info("Not a valid action sequence. Only published custom info (no movement).")
            return

        # 如果正在执行旧序列，先停掉
        self.cancel_timer()
        self.stop()

        self._queue = queue
        self.get_logger().info(f"Start sequence with {len(self._queue)} steps")
        self.run_next_step()


def main(args=None):
    rclpy.init(args=args)
    node = QRCodeControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cancel_timer()
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

