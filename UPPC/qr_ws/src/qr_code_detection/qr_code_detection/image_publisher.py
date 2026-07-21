#!/usr/bin/env python3
"""
image_publisher.py (可运行时切换二维码)
功能：
- 循环发布一张图片到 /camera/image_raw，模拟相机
- 订阅 /qr_select (std_msgs/String)，接收 'Front'/'Back'/'Left'/'Right'
  然后立刻切换要发布的图片
"""

import os
import threading
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge


class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')

        # 参数
        self.declare_parameter('images_dir', os.path.expanduser('~/qr_ws/qrcodes'))
        self.declare_parameter('topic', '/camera/image_raw')
        self.declare_parameter('publish_hz', 5.0)
        self.declare_parameter('select_topic', '/qr_select')
        self.declare_parameter('default_qr', 'Front')  # 启动默认二维码

        self.images_dir = str(self.get_parameter('images_dir').value)
        self.topic = str(self.get_parameter('topic').value)
        self.hz = float(self.get_parameter('publish_hz').value)
        self.select_topic = str(self.get_parameter('select_topic').value)
        self.default_qr = str(self.get_parameter('default_qr').value)

        if not os.path.isdir(self.images_dir):
            self.get_logger().error(f"images_dir 不存在：{self.images_dir}")
            raise FileNotFoundError(self.images_dir)

        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, self.topic, 10)
        self.sub = self.create_subscription(String, self.select_topic, self.on_select, 10)

        self.lock = threading.Lock()
        self.cv_img = None
        self.current_name = None

        # 先加载默认二维码
        self.set_image(self.default_qr)

        self.timer = self.create_timer(1.0 / self.hz, self.on_timer)
        self.get_logger().info(f"Publishing -> {self.topic} @ {self.hz} Hz")
        self.get_logger().info(f"Select QR by: ros2 topic pub --once {self.select_topic} std_msgs/msg/String \"{{data: 'Left'}}\"")

    def image_path(self, name: str) -> str:
        # 允许用户发 'Front' 或 'Front.png'
        if name.lower().endswith('.png') or name.lower().endswith('.jpg') or name.lower().endswith('.jpeg'):
            filename = name
        else:
            filename = f"{name}.png"
        return os.path.join(self.images_dir, filename)

    def set_image(self, name: str):
        path = self.image_path(name)
        if not os.path.exists(path):
            self.get_logger().warn(f"找不到二维码图片：{path}（请确认存在 Front/Back/Left/Right.png）")
            return

        img = cv2.imread(path, cv2.IMREAD_COLOR)
        if img is None:
            self.get_logger().warn(f"读取失败：{path}")
            return

        with self.lock:
            self.cv_img = img
            self.current_name = os.path.basename(path)

        self.get_logger().info(f"Switched QR image -> {path}")

    def on_select(self, msg: String):
        name = (msg.data or "").strip()
        if not name:
            return
        self.set_image(name)

    def on_timer(self):
        with self.lock:
            if self.cv_img is None:
                return
            img = self.cv_img

        ros_img = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        ros_img.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(ros_img)


def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

