# -*- coding: utf-8 -*-
"""
激光雷达里程计中继节点 (odom_relay_node)

职责非常简单——订阅外部 LIO 里程计话题 → 统一转发到 /odometry
因为 serial_bridge 只订阅 /odometry，而不同的 LIO 包发布的话题名不一样
（FAST-LIO2 发布 /livox/odometry），这个节点就是"统一接口"。

外部包路径：/home/shijue2/lidar/ws_livox/install/livox_ros_driver2
（驱动/LIO 的 launch 由 bringup 通过 IncludeLaunchDescription 引入，见 launch 文件）
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy  # QoS 配置

from nav_msgs.msg import Odometry   # 里程计消息类型


class OdomRelay(Node):
    def __init__(self) -> None:
        super().__init__("lidar_odom_relay")     # 节点名

        # 可配参数（通过 YAML / launch 传参）
        self.declare_parameter("input_odom_topic", "/livox/odometry")   # 输入话题 = LIO 输出的
        self.declare_parameter("output_odom_topic", "/odometry")        # 输出话题 = serial_bridge 订阅的
        self.declare_parameter("override_frame_id", "")                 # 空 = 保持原 frame 名
        self.declare_parameter("override_child_frame_id", "")

        self.out_frame = self.get_parameter("override_frame_id").value
        self.out_child = self.get_parameter("override_child_frame_id").value

        # BEST_EFFORT QoS：里程计频率高数据量大，丢了不重传，只要最新帧
        qos = QoSProfile(depth=10,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)

        # 发布 → 转发到 /odometry
        self.pub = self.create_publisher(
            Odometry, self.get_parameter("output_odom_topic").value, qos)

        # 订阅 → 收 LIO 发来的原始里程计
        self.create_subscription(
            Odometry, self.get_parameter("input_odom_topic").value,
            self._on_odom, qos)                                    # 收到 → 调 _on_odom

        self.get_logger().info(
            f"lidar_odom_relay started: "
            f"{self.get_parameter('input_odom_topic').value} -> "
            f"{self.get_parameter('output_odom_topic').value}")

    def _on_odom(self, msg: Odometry) -> None:
        """
        收到 LIO 里程计 → 可选覆盖 frame 名 → 直接转发
        不做坐标变换、不做滤波——纯转发，延迟最小
        Python 的 if self.out_frame: → 如果字符串非空，执行下面的赋值
        """
        if self.out_frame:                       # 如果配置了覆盖 frame_id
            msg.header.frame_id = self.out_frame
        if self.out_child:                       # 如果配置了覆盖 child_frame_id
            msg.child_frame_id = self.out_child
        self.pub.publish(msg)                    # 直接转发原始消息（零拷贝）


def main(args=None) -> None:
    """ROS2 入口"""
    rclpy.init(args=args)
    node = OdomRelay()
    try:
        rclpy.spin(node)                         # ROS2 死循环
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == "__main__":
    main()
