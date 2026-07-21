#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import ReliabilityPolicy, QoSProfile

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String

from kfs_msgs.msg import YoloResult, YoloObject
import message_filters
import cv2
import numpy as np
from ultralytics import YOLO
import time
import math

class YoloD435iNode(Node):
    def __init__(self):
        super().__init__('kfs_seg')

        # --- 参数设置 ---
        self.declare_parameter('model_path', '/home/ek/RC2026/src/kfs_seg/kfs_seg/model/best.pt') 
        self.declare_parameter('conf_thres', 0.5)
        self.conf_thres = self.get_parameter('conf_thres').value
        model_path = self.get_parameter('model_path').value

        # --- 初始化模型 ---
        self.get_logger().info(f'Loading YOLO model: {model_path}...')
        try:
            self.model = YOLO(model_path)
            self.get_logger().info(f'模型包含的类别: {self.model.names}')
        except Exception as e:
            self.get_logger().error(f"模型加载失败: {e}")

        # --- 订阅 ---
        self.color_sub = message_filters.Subscriber(
            self, Image, '/camera/color/image_raw',
            qos_profile=qos_profile_sensor_data
        )

        self.depth_sub = message_filters.Subscriber(
            self, Image, '/camera/aligned_depth_to_color/image_raw',
            qos_profile=qos_profile_sensor_data
        )

        self.info_sub = self.create_subscription(
            CameraInfo, '/camera/color/camera_info', 
            self.info_callback,
            qos_profile=qos_profile_sensor_data
        )

        # --- 发布 ---
        self.detect_pub = self.create_publisher(YoloResult, '/yolo/result', 10)
        self.result_img_pub = self.create_publisher(Image, '/yolo/image_result', 10)

        # 时间同步器
        self.ts = message_filters.ApproximateTimeSynchronizer([self.color_sub, self.depth_sub], 10, 0.1)
        self.ts.registerCallback(self.listener_callback)

        # 相机内参
        self.intrinsics = None
        
        # FPS计算
        self.prev_time = time.time()

        self.get_logger().info('Yolo Node Started. Waiting for images...')

    def info_callback(self, msg):
        if self.intrinsics is None:
            self.intrinsics = {'fx': msg.k[0], 'fy': msg.k[4], 'ppx': msg.k[2], 'ppy': msg.k[5]}
            self.get_logger().info(f"Camera Intrinsics Loaded: {self.intrinsics}")

    def imgmsg_to_cv2(self, img_msg, desired_encoding="bgr8"):
        """ 手动转换 ROS -> CV2 """
        if img_msg.encoding == "bgr8" or img_msg.encoding == "rgb8":
            dtype = np.uint8; n_channels = 3
        elif img_msg.encoding == "16UC1":
            dtype = np.uint16; n_channels = 1
        else: return None
        
        dtype = np.dtype(dtype).newbyteorder('>' if img_msg.is_bigendian else '<')
        img_buf = np.asarray(img_msg.data, dtype=dtype)
        
        if n_channels == 1:
            im = np.ndarray(shape=(img_msg.height, img_msg.width), dtype=dtype, buffer=img_buf)
        else:
            im = np.ndarray(shape=(img_msg.height, img_msg.width, n_channels), dtype=dtype, buffer=img_buf)
            
        if img_msg.encoding == "rgb8" and desired_encoding == "bgr8":
            im = cv2.cvtColor(im, cv2.COLOR_RGB2BGR)
        return im

    def cv2_to_imgmsg(self, cv_image):
        """ 手动转换 CV2 -> ROS """
        img_msg = Image()
        img_msg.height = cv_image.shape[0]
        img_msg.width = cv_image.shape[1]
        img_msg.encoding = "bgr8"
        img_msg.is_bigendian = 0
        img_msg.step = cv_image.shape[1] * 3
        img_msg.data = cv_image.tobytes()
        return img_msg

    def calculate_yaw_from_mask(self, mask_pts):
        if len(mask_pts) < 10: return 0.0
        mean, eigenvectors = cv2.PCACompute(mask_pts.reshape(-1, 2).astype(np.float32), mean=None)
        axis = eigenvectors[0]
        return math.atan2(axis[1], axis[0])

    def deproject_pixel_to_point(self, u, v, depth):
        if self.intrinsics is None: return 0, 0, 0
        x = (u - self.intrinsics['ppx']) * depth / self.intrinsics['fx']
        y = (v - self.intrinsics['ppy']) * depth / self.intrinsics['fy']
        return x, y, depth

    def listener_callback(self, color_msg, depth_msg):
        self.get_logger().info("Callback Triggered!")
        if self.intrinsics is None: return

        # 1. 转 CV2
        cv_image = self.imgmsg_to_cv2(color_msg, "bgr8")
        cv_depth = self.imgmsg_to_cv2(depth_msg, "16UC1")
        if cv_image is None or cv_depth is None: return

        # 2. 推理
        results = self.model(cv_image, conf=self.conf_thres, verbose=False)
        result = results[0]
        
        # 3. 准备 ROS 消息
        res_msg = YoloResult()
        res_msg.header = color_msg.header

        if result.boxes:
            masks = result.masks.xy if result.masks else None
            h_img, w_img = cv_depth.shape

            for i, box in enumerate(result.boxes):
                label = result.names[int(box.cls[0])]
                conf = float(box.conf[0])
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)

                cx_safe = max(0, min(cx, w_img - 1))
                cy_safe = max(0, min(cy, h_img - 1))
                d_raw = cv_depth[cy_safe, cx_safe]
                
                if d_raw == 0:
                    roi = cv_depth[max(0, cy_safe-2):min(h_img, cy_safe+3), 
                                   max(0, cx_safe-2):min(w_img, cx_safe+3)]
                    valid = roi[roi > 0]
                    if len(valid) > 0: d_raw = np.median(valid)
                
                depth_m = d_raw * 0.001
                
                if depth_m > 0:
                    rx, ry, rz = self.deproject_pixel_to_point(cx, cy, depth_m)
                    
                    yaw = 0.0
                    if masks is not None and len(masks) > i:
                        yaw = self.calculate_yaw_from_mask(masks[i])

                    # 填充消息
                    obj = YoloObject()
                    obj.class_name = str(label)
                    obj.score = float(conf)
                    obj.x = float(rx); obj.y = float(ry); obj.z = float(rz)
                    obj.yaw = float(yaw)
                    res_msg.objects.append(obj)

                    # 画图
                    cv2.rectangle(cv_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                    info_txt = f"{label} D:{depth_m:.2f}m Y:{math.degrees(yaw):.0f}"
                    cv2.putText(cv_image, info_txt, (int(x1), int(y1)-5), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    print(f"[{i}] {label} XYZ:({rx:.2f}, {ry:.2f}, {rz:.2f}) Yaw:{math.degrees(yaw):.1f}")

        # 发布结果数据
        self.detect_pub.publish(res_msg)

        # 发布结果图像给 Rviz
        try:
            out_img_msg = self.cv2_to_imgmsg(cv_image)
            out_img_msg.header = color_msg.header
            self.result_img_pub.publish(out_img_msg)
        except Exception as e:
            self.get_logger().warn(f"Image Pub Error: {e}")

        # 显示 FPS
        curr_time = time.time()
        fps = 1.0 / (curr_time - self.prev_time)
        self.prev_time = curr_time
        cv2.putText(cv_image, f"FPS: {fps:.1f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
        
        cv2.imshow("YOLOv8-Seg", cv_image)
        if cv2.waitKey(1) == 27:
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = YoloD435iNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()