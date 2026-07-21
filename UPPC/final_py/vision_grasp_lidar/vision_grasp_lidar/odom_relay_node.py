# -*- coding: utf-8 -*-
"""
婵€鍏夐浄杈鹃噷绋嬭涓户鑺傜偣 (odom_relay)
- 璁㈤槄澶栭儴 livox_ros_driver2 / LIO 閲岀▼璁¤瘽棰橈紝
  缁熶竴杞彂鍒� /odometry锛屼緵 serial_bridge 鎸佺画鍚戜笅浣嶆満鍙戦€併€�
- 澶栭儴鍖呰矾寰勶細/home/shijue2/lidar/ws_livox/install/livox_ros_driver2
  锛堝叾椹卞姩/閲岀▼璁� launch 鐢� bringup 閫氳繃 IncludeLaunchDescription 寮曞叆锛岃 launch 鏂囦欢锛�
- 鏈妭鐐逛粎鍋氳瘽棰樿浆鍙� + 鍙€夌殑 frame 鍚嶈鐩栵紝
  涓嶉噸澶嶅疄鐜伴┍鍔ㄩ€昏緫锛涙弧瓒�"浣跨敤澶栭儴鍖�"鐨勮姹傘€�
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry


class OdomRelay(Node):
    def __init__(self) -> None:
        super().__init__("lidar_odom_relay")

        self.declare_parameter("input_odom_topic", "/livox/odometry")
        self.declare_parameter("output_odom_topic", "/odometry")
        self.declare_parameter("override_frame_id", "")     # 绌�=淇濇寔鍘熷 frame
        self.declare_parameter("override_child_frame_id", "")

        self.out_frame = self.get_parameter("override_frame_id").value
        self.out_child = self.get_parameter("override_child_frame_id").value

        qos = QoSProfile(depth=10,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)

        self.pub = self.create_publisher(
            Odometry, self.get_parameter("output_odom_topic").value, qos)
        self.create_subscription(
            Odometry, self.get_parameter("input_odom_topic").value,
            self._on_odom, qos)

        self.get_logger().info(
            f"lidar_odom_relay started: "
            f"{self.get_parameter('input_odom_topic').value} -> "
            f"{self.get_parameter('output_odom_topic').value}")

    def _on_odom(self, msg: Odometry) -> None:
        if self.out_frame:
            msg.header.frame_id = self.out_frame
        if self.out_child:
            msg.child_frame_id = self.out_child
        self.pub.publish(msg)


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
