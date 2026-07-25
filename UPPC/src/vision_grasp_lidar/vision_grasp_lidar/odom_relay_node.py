# -*- coding: utf-8 -*-
"""
激光雷达里程计中继节点 (odom_relay)
- 订阅外部 livox_ros_driver2 / LIO 里程计话题，
  统一转发到 /odometry，供 serial_bridge 持续向下位机发送。
- 支持里程计实时归零：收到 /odom_reset(Bool=True) 时，
  将当前位姿记录为新原点，后续发布数据均相对于该原点。
  归零操作即时生效（<1ms），无需重启 SLAM 节点。

归零数学原理：
    设归零时刻的位姿为 P_off（位置 t_off，旋转四元数 q_off）
    对后续每帧位姿 P_cur（位置 t_cur，旋转 q_cur）：
        t_out = R(q_off)^T · (t_cur - t_off)     # 相对位置，转到归零坐标系
        q_out = q_off_inv * q_cur                 # 相对旋转
    其中 q_off_inv = (-x, -y, -z, w)（单位四元数共轭即逆）
    速度（twist）在 child_frame（机体系）中表达，无需变换，直接透传。
"""

import math
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry
from std_msgs.msg import Bool


class OdomRelay(Node):
    def __init__(self) -> None:
        super().__init__("lidar_odom_relay")

        self.declare_parameter("input_odom_topic",       "/livox/odometry")
        self.declare_parameter("output_odom_topic",      "/odometry")
        self.declare_parameter("odom_reset_topic",       "/odom_reset")
        self.declare_parameter("override_frame_id",      "")   # 空=保持原始 frame
        self.declare_parameter("override_child_frame_id", "")

        self.out_frame = self.get_parameter("override_frame_id").value
        self.out_child = self.get_parameter("override_child_frame_id").value

        # ── 归零状态 ──────────────────────────────────────────────────────────
        # _offset 为 None 时直接透传（系统启动后未归零的默认状态）
        self._offset: Optional[Odometry] = None
        # 缓存最新一帧，用于归零时捕获当前位姿
        self._latest: Optional[Odometry] = None

        # ── QoS ─────────────────────────────────────────────────────────────
        qos = QoSProfile(depth=10,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)

        # ── 发布者 ───────────────────────────────────────────────────────────
        self.pub = self.create_publisher(
            Odometry, self.get_parameter("output_odom_topic").value, qos)

        # ── 订阅者 ───────────────────────────────────────────────────────────
        self.create_subscription(
            Odometry,
            self.get_parameter("input_odom_topic").value,
            self._on_odom, qos)

        self.create_subscription(
            Bool,
            self.get_parameter("odom_reset_topic").value,
            self._on_reset, 1)

        self.get_logger().info(
            f"lidar_odom_relay started: "
            f"{self.get_parameter('input_odom_topic').value} -> "
            f"{self.get_parameter('output_odom_topic').value}  "
            f"reset_topic={self.get_parameter('odom_reset_topic').value}")

    # ═══════════════════════════════════════════════════════════ 归零处理
    def _on_reset(self, msg: Bool) -> None:
        """收到归零指令（data=True）时，将当前位姿记为新原点。"""
        if not msg.data:
            return
        if self._latest is None:
            self.get_logger().warn(
                "收到归零指令，但尚未收到任何里程计帧，忽略。")
            return
        self._offset = self._latest
        p = self._offset.pose.pose.position
        q = self._offset.pose.pose.orientation
        self.get_logger().info(
            f"里程计归零完成  原点位置=({p.x:.3f}, {p.y:.3f}, {p.z:.3f})  "
            f"原点四元数=({q.x:.4f}, {q.y:.4f}, {q.z:.4f}, {q.w:.4f})")

    # ═══════════════════════════════════════════════════════════ 里程计回调
    def _on_odom(self, msg: Odometry) -> None:
        """缓存最新帧，按需应用坐标偏移后转发。"""
        self._latest = msg

        if self._offset is None:
            # 未归零，直接透传
            out = msg
        else:
            out = self._apply_offset(msg)

        # frame_id 覆盖（可选）
        if self.out_frame:
            out.header.frame_id = self.out_frame
        if self.out_child:
            out.child_frame_id = self.out_child

        self.pub.publish(out)

    # ═══════════════════════════════════════════════════════════ 坐标变换
    def _apply_offset(self, msg: Odometry) -> Odometry:
        """将 msg 的位姿变换到以 _offset 为原点的坐标系中。

        位置变换：t_out = R(q_off)^T · (t_cur - t_off)
        姿态变换：q_out = q_off_inv * q_cur
        速度（twist）：在 child_frame 中表达，无需变换，直接复制。
        """
        off = self._offset
        op  = off.pose.pose.position
        oq  = off.pose.pose.orientation

        cp  = msg.pose.pose.position
        cq  = msg.pose.pose.orientation

        # 位置差（在全局坐标系中）
        dx = cp.x - op.x
        dy = cp.y - op.y
        dz = cp.z - op.z

        # 将位置差旋转到归零坐标系（乘以 q_off 的逆旋转矩阵，即 R^T）
        rx, ry, rz = self._rotate_inv(dx, dy, dz, oq.x, oq.y, oq.z, oq.w)

        # 相对姿态：q_out = q_off_inv * q_cur
        qox, qoy, qoz, qow = self._quat_mul(
            -oq.x, -oq.y, -oq.z, oq.w,    # q_off 的逆（共轭）
             cq.x,  cq.y,  cq.z, cq.w)    # q_cur

        # 构造输出消息
        out = Odometry()
        out.header            = msg.header
        out.child_frame_id    = msg.child_frame_id

        out.pose.pose.position.x    = rx
        out.pose.pose.position.y    = ry
        out.pose.pose.position.z    = rz
        out.pose.pose.orientation.x = qox
        out.pose.pose.orientation.y = qoy
        out.pose.pose.orientation.z = qoz
        out.pose.pose.orientation.w = qow

        # 协方差直接复制
        out.pose.covariance  = msg.pose.covariance

        # 速度在机体系中表达，归零不影响速度，直接复制
        out.twist = msg.twist

        return out

    @staticmethod
    def _rotate_inv(dx: float, dy: float, dz: float,
                    qx: float, qy: float, qz: float, qw: float
                    ) -> Tuple[float, float, float]:
        """将向量 (dx, dy, dz) 绕四元数 (qx,qy,qz,qw) 的 **逆方向** 旋转。
        等价于乘以旋转矩阵的转置 R^T，即用共轭四元数旋转。

        推导：
            q_inv = (-qx, -qy, -qz, qw)
            旋转矩阵 R(q_inv) 的各行：
                r0 = [1-2(qiy²+qiz²),  2(qix*qiy-qiz*qiw),  2(qix*qiz+qiy*qiw)]
                r1 = [2(qix*qiy+qiz*qiw), 1-2(qix²+qiz²),   2(qiy*qiz-qix*qiw)]
                r2 = [2(qix*qiz-qiy*qiw), 2(qiy*qiz+qix*qiw), 1-2(qix²+qiy²)]
            其中 qix=-qx, qiy=-qy, qiz=-qz, qiw=qw
            化简后与 R(q)^T 的各列相同。
        """
        # 共轭四元数分量
        qix, qiy, qiz, qiw = -qx, -qy, -qz, qw

        rx = ((1 - 2*(qiy*qiy + qiz*qiz)) * dx
              + 2*(qix*qiy - qiz*qiw) * dy
              + 2*(qix*qiz + qiy*qiw) * dz)

        ry = (2*(qix*qiy + qiz*qiw) * dx
              + (1 - 2*(qix*qix + qiz*qiz)) * dy
              + 2*(qiy*qiz - qix*qiw) * dz)

        rz = (2*(qix*qiz - qiy*qiw) * dx
              + 2*(qiy*qiz + qix*qiw) * dy
              + (1 - 2*(qix*qix + qiy*qiy)) * dz)

        return rx, ry, rz

    @staticmethod
    def _quat_mul(ax: float, ay: float, az: float, aw: float,
                  bx: float, by: float, bz: float, bw: float
                  ) -> Tuple[float, float, float, float]:
        """四元数乘法 a * b，使用 Hamilton 乘积公式。
        四元数约定：q = (x, y, z, w)，w 为实部。
        """
        return (
            aw*bx + ax*bw + ay*bz - az*by,
            aw*by - ax*bz + ay*bw + az*bx,
            aw*bz + ax*by - ay*bx + az*bw,
            aw*bw - ax*bx - ay*by - az*bz,
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdomRelay()
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

