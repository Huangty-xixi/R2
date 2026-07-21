#!/usr/bin/env python3
"""
qr_decoder.py (带窗口显示版)
功能：
1) 订阅 /camera/image_raw
2) 识别二维码（pyzbar）
3) 每帧发布 /qrcode/text
4) 弹出 OpenCV 窗口显示当前画面，并叠加：
   - 二维码框
   - 识别到的文字
"""

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

import cv2
from pyzbar.pyzbar import decode


class QRDecoder(Node):
    def __init__(self):
        super().__init__('qr_decoder')

        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('output_topic', '/qrcode/text')

        # 识别增强参数
        self.declare_parameter('resize_scale', 2.0)
        self.declare_parameter('use_threshold', True)

        # 显示相关参数
        self.declare_parameter('show_window', True)
        self.declare_parameter('window_name', 'QR Decoder')
        self.declare_parameter('window_width', 640)
        self.declare_parameter('window_height', 480)

        self.image_topic = str(self.get_parameter('image_topic').value)
        self.output_topic = str(self.get_parameter('output_topic').value)

        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, self.image_topic, self.on_image, 10)
        self.pub = self.create_publisher(String, self.output_topic, 10)

        self.last_text = None

        self.show_window = bool(self.get_parameter('show_window').value)
        self.window_name = str(self.get_parameter('window_name').value)
        self.window_width = int(self.get_parameter('window_width').value)
        self.window_height = int(self.get_parameter('window_height').value)

        if self.show_window:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.window_name, self.window_width, self.window_height)

        self.get_logger().info(f"QRDecoder subscribe: {self.image_topic}  publish: {self.output_topic}")

    def on_image(self, msg: Image):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

            scale = float(self.get_parameter('resize_scale').value)
            if scale and scale != 1.0:
                gray_proc = cv2.resize(gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
            else:
                gray_proc = gray

            use_th = bool(self.get_parameter('use_threshold').value)
            if use_th:
                _, gray_proc = cv2.threshold(gray_proc, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

            results = decode(gray_proc)

            # 用于显示的叠加信息（默认无识别）
            overlay_text = "No QR detected"
            detected_text = None

            if results:
                # 只取第一个二维码
                r = results[0]
                detected_text = r.data.decode('utf-8', errors='ignore').strip()

                if detected_text:
                    # 每帧发布（保证控制节点随时能收到）
                    out = String()
                    out.data = detected_text
                    self.pub.publish(out)

                    # 日志只在变化时打印
                    if detected_text != self.last_text:
                        self.get_logger().info(f"Detected QR: {detected_text}")
                        self.last_text = detected_text

                    overlay_text = f"Detected: {detected_text}"

                # 画框：pyzbar 给的是在处理过的图(gray_proc)上的坐标
                # 这里为了简单显示：直接在 bgr 上画一个“近似框”（按比例缩放回去）
                rect = r.rect  # (x, y, w, h) on processed image
                if rect:
                    x, y, w, h = rect.left, rect.top, rect.width, rect.height
                    # 反缩放回原图坐标
                    inv = 1.0 / scale if (scale and scale != 0.0) else 1.0
                    x0 = int(x * inv)
                    y0 = int(y * inv)
                    x1 = int((x + w) * inv)
                    y1 = int((y + h) * inv)
                    cv2.rectangle(bgr, (x0, y0), (x1, y1), (0, 255, 0), 2)

                    if detected_text:
                        cv2.putText(
                            bgr, detected_text, (x0, max(20, y0 - 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2
                        )

            # 左上角状态文字
            cv2.putText(
                bgr, overlay_text, (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2
            )

            # 显示窗口
            if self.show_window:
                cv2.imshow(self.window_name, bgr)
                # waitKey 必须要有，否则窗口不刷新
                cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"decode error: {e}")

    def destroy_node(self):
        if self.show_window:
            try:
                cv2.destroyAllWindows()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = QRDecoder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

