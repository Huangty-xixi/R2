# -*- coding: utf-8 -*-
"""
debug_vis_node.py  鈥斺€�  妫€娴嬪彲瑙嗗寲璋冭瘯鑺傜偣
==========================================
鐙珛杩愯 YOLOv8 鎺ㄧ悊锛堜笌 detector_node 瀹屽叏瑙ｈ€︼紝浠呯敤浜庡彲瑙嗗寲锛夛紝
灏嗘爣娉ㄥ悗鐨勫浘鍍忓彂甯冨埌 /debug_image锛屽彲鍦� rqt_image_view 涓疄鏃舵煡鐪嬨€�

鍙鍖栧唴瀹癸細
    路 鎵€鏈夌洰鏍囩殑 YOLO 鍖呭洿妗嗭紙缁�=鏈€浼樼洰鏍囷紝钃�=鍏朵粬鐩爣锛�
    路 鍒嗗壊鎺╄啘杞粨
    路 鏈€浼樼洰鏍囦腑蹇冨崄瀛楃嚎 + 涓績鍍忕礌鍧愭爣 (u, v)
    路 鏈€浼樼洰鏍囩浉鏈哄潗鏍� (x, y, z) 鍗曚綅 m
    路 IN RANGE / OUT OF RANGE 鐘舵€佹彁绀�
    路 瀹炴椂鎺ㄧ悊甯х巼 FPS
    路 鍙充笅瑙掔浉鏈哄潗鏍囩郴鍥句緥

鐩告満鍧愭爣绯伙紙ROS optical frame锛屼笌 deproject_pixel 涓€鑷达級锛�
    X 鈫�  鍚戝彸锛坲 澧炲ぇ鏂瑰悜锛�  姝� = 鐩爣鍦ㄧ浉鏈哄彸渚э紝璐� = 宸︿晶
    Y 鈫�  鍚戜笅锛坴 澧炲ぇ鏂瑰悜锛�  姝� = 鐩爣鍦ㄧ浉鏈轰笅鏂癸紝璐� = 涓婃柟
    Z 鈼�  鍚戝墠锛堝厜杞�/娣卞害锛�   姝� = 鐩爣鍦ㄧ浉鏈哄墠鏂癸紙濮嬬粓姝ｅ€硷級

浣跨敤鏂规硶锛�
    # 鎶婃湰鏂囦欢鏀惧埌 vision_grasp_camera/vision_grasp_camera/ 涓嬶紝
    # 鍦� setup.py 鐨� console_scripts 閲屽姞涓€琛岋細
    #     'debug_vis = vision_grasp_camera.debug_vis_node:main',
    # 鐒跺悗閲嶆柊 colcon build锛屽啀鎵ц锛�
    ros2 run vision_grasp_camera debug_vis \
      --ros-args --params-file ~/RC2026/src/vision_grasp_bringup/config/camera_params.yaml

    # 鍦� rqt 涓煡鐪嬶細
    rqt
    # Plugins 鈫� Visualization 鈫� Image View 鈫� 閫夋嫨 /debug_image
"""

import time
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import message_filters
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo

from .geometry_utils import (
    sample_center_depth,
    deproject_pixel,
    mask_centroid,
)

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None

# 鈹€鈹€鈹€ 棰滆壊甯搁噺 (BGR) 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
_GREEN  = (50,  210,  50)    # 鏈€浼樼洰鏍囨
_BLUE   = (200, 120,  30)    # 鍏朵粬鐩爣妗�
_RED    = (30,   30, 220)    # 鍗佸瓧绾�
_CYAN   = (220, 200,   0)    # 鍧愭爣鏂囧瓧
_YELLOW = (0,   200, 220)    # OUT OF RANGE
_WHITE  = (255, 255, 255)
_BLACK  = (0,     0,   0)
_GRAY   = (160, 160, 160)


class DebugVisNode(Node):
    def __init__(self) -> None:
        super().__init__("debug_vis")

        # 鈹€鈹€ 鍙傛暟澹版槑锛堜笌 camera_params.yaml 涓� camera_detector 娈靛悓鍚嶏紝鍙叡鐢級 鈹€鈹€
        self.declare_parameter("color_topic",
                               "/camera/camera/color/image_raw")
        self.declare_parameter("depth_topic",
                               "/camera/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("camera_info_topic",
                               "/camera/camera/color/camera_info")
        self.declare_parameter("debug_image_topic",  "/debug_image")
        self.declare_parameter("model_path",
                               "/home/shijue2/RC2026/src/vision_grasp_camera/models/best.pt")
        self.declare_parameter("device",             "cpu")
        self.declare_parameter("conf_threshold",     0.4)
        self.declare_parameter("iou_threshold",      0.4)
        self.declare_parameter("imgsz",              640)
        self.declare_parameter("target_class_ids",   [""])   # [""] = 鎺ュ彈鍏ㄩ儴绫诲埆
        self.declare_parameter("depth_scale",        0.001)
        self.declare_parameter("depth_window",       5)
        self.declare_parameter("min_depth_m",        0.05)
        self.declare_parameter("max_depth_m",        5.0)
        self.declare_parameter("range_min_m",        0.50)
        self.declare_parameter("range_max_m",        1.00)

        # 鈹€鈹€ 璇诲彇鍙傛暟 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
        self.conf         = float(self.get_parameter("conf_threshold").value)
        self.iou          = float(self.get_parameter("iou_threshold").value)
        self.imgsz        = int(self.get_parameter("imgsz").value)
        self.device       = self.get_parameter("device").value
        self.depth_scale  = float(self.get_parameter("depth_scale").value)
        self.depth_window = int(self.get_parameter("depth_window").value)
        self.min_depth    = float(self.get_parameter("min_depth_m").value)
        self.max_depth    = float(self.get_parameter("max_depth_m").value)
        self.range_min    = float(self.get_parameter("range_min_m").value)
        self.range_max    = float(self.get_parameter("range_max_m").value)
        raw = self.get_parameter("target_class_ids").value
        self.target_class_names = [s for s in (raw or []) if s]   # 绌哄垪琛�=鎺ュ彈鍏ㄩ儴

        # 鈹€鈹€ 鍐呴儴鐘舵€� 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
        self.fx = self.fy = self.cx = self.cy = None
        self.bridge   = CvBridge()
        self._last_t  = time.monotonic()
        self._fps     = 0.0

        # 鈹€鈹€ 妯″瀷鍔犺浇 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
        self.model = None
        self._load_model()

        # 鈹€鈹€ 鍙戝竷鑰� 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
        self.debug_pub = self.create_publisher(
            Image, self.get_parameter("debug_image_topic").value, 5)

        # 鈹€鈹€ 璁㈤槄鑰� 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
        self.create_subscription(
            CameraInfo,
            self.get_parameter("camera_info_topic").value,
            self._on_camera_info, 10)

        sensor_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST)

        color_sub = message_filters.Subscriber(
            self, Image,
            self.get_parameter("color_topic").value,
            qos_profile=sensor_qos)
        depth_sub = message_filters.Subscriber(
            self, Image,
            self.get_parameter("depth_topic").value,
            qos_profile=sensor_qos)
        self._sync = message_filters.ApproximateTimeSynchronizer(
            [color_sub, depth_sub], queue_size=5, slop=0.05)
        self._sync.registerCallback(self._on_frame)

        self.get_logger().info(
            f"debug_vis 鍚姩  "
            f"model={self.get_parameter('model_path').value}  "
            f"filter={self.target_class_names or '鍏ㄩ儴绫诲埆'}  "
            f"range=[{self.range_min}, {self.range_max}] m")

    # 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲 鍒濆鍖�
    def _load_model(self) -> None:
        path = self.get_parameter("model_path").value
        if not path:
            self.get_logger().warn("model_path 鏈厤缃紝璺宠繃妯″瀷鍔犺浇")
            return
        if YOLO is None:
            self.get_logger().error("ultralytics 鏈畨瑁咃紙pip install ultralytics锛�")
            return
        try:
            self.model = YOLO(path)
            self.get_logger().info(f"妯″瀷宸插姞杞斤細{path}")
        except Exception as exc:                    # noqa: BLE001
            self.get_logger().error(f"妯″瀷鍔犺浇澶辫触锛歿exc")

    # 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲 鍥炶皟
    def _on_camera_info(self, msg: CameraInfo) -> None:
        """浠� camera_info 璇诲彇鍐呭弬锛岀敤浜庡弽鎶曞奖璁＄畻鐩告満鍧愭爣銆�"""
        self.fx, self.fy = msg.k[0], msg.k[4]
        self.cx, self.cy = msg.k[2], msg.k[5]

    def _on_frame(self, color_msg: Image, depth_msg: Image) -> None:
        """涓诲洖璋冿細鎺ㄧ悊 鈫� 鏍囨敞 鈫� 鍙戝竷銆�"""
        if self.model is None:
            return

        # FPS锛堝抚闂存椂闂达級
        now = time.monotonic()
        dt  = now - self._last_t
        self._last_t = now
        self._fps = 1.0 / dt if dt > 1e-6 else 0.0

        color = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        h, w  = color.shape[:2]
        vis   = color.copy()

        # 鈹€鈹€ YOLO 鎺ㄧ悊 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
        results = self.model.predict(
            source=color,
            conf=self.conf, iou=self.iou,
            imgsz=self.imgsz, device=self.device,
            verbose=False)

        best_idx  = None
        best_conf = -1.0
        n_det     = 0

        if results and results[0].boxes is not None and len(results[0].boxes):
            res      = results[0]
            names    = res.names
            cls_arr  = res.boxes.cls.cpu().numpy().astype(int)
            conf_arr = res.boxes.conf.cpu().numpy()
            xyxy_arr = res.boxes.xyxy.cpu().numpy().astype(int)
            masks_np = (res.masks.data.cpu().numpy()
                        if res.masks is not None else None)   # (n, mh, mw)

            # 鍏堟壘鏈€浼樼洰鏍囷紙缃俊搴︽渶楂樹笖閫氳繃绫诲埆杩囨护锛�
            for i in range(len(cls_arr)):
                cname = names.get(int(cls_arr[i]), str(cls_arr[i]))
                if self.target_class_names and cname not in self.target_class_names:
                    continue
                n_det += 1
                if float(conf_arr[i]) > best_conf:
                    best_conf = float(conf_arr[i])
                    best_idx  = i

            # 缁樺埗鎵€鏈夐€氳繃杩囨护鐨勭洰鏍�
            for i in range(len(cls_arr)):
                cid   = int(cls_arr[i])
                cname = names.get(cid, str(cid))
                if self.target_class_names and cname not in self.target_class_names:
                    continue

                x1, y1, x2, y2 = xyxy_arr[i]
                is_best = (i == best_idx)
                col     = _GREEN if is_best else _BLUE
                thick   = 3 if is_best else 1

                # 鎺╄啘杞粨
                if masks_np is not None:
                    m = masks_np[i]
                    if m.shape != (h, w):
                        m = cv2.resize(m.astype(np.float32), (w, h),
                                       interpolation=cv2.INTER_NEAREST)
                    contours, _ = cv2.findContours(
                        (m > 0.5).astype(np.uint8),
                        cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    cv2.drawContours(vis, contours, -1, col, 2)

                # YOLO 鍖呭洿妗�
                cv2.rectangle(vis, (x1, y1), (x2, y2), col, thick)

                # 鏍囩鑳屾櫙 + 鏂囧瓧
                label = f"{cname}  {conf_arr[i]:.2f}"
                (tw, th), _ = cv2.getTextSize(
                    label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
                cv2.rectangle(vis, (x1, y1 - th - 8), (x1 + tw + 6, y1), col, -1)
                cv2.putText(vis, label, (x1 + 3, y1 - 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, _BLACK, 1, cv2.LINE_AA)

                # 鈹€鈹€ 鏈€浼樼洰鏍囷細鍗佸瓧绾� + 娣卞害 + 鍧愭爣 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
                if is_best:
                    # 鎺╄啘璐ㄥ績锛堜笌 detector_node 瀹屽叏涓€鑷达級
                    if masks_np is not None:
                        m_bin = masks_np[i]
                        if m_bin.shape != (h, w):
                            m_bin = cv2.resize(
                                m_bin.astype(np.float32), (w, h),
                                interpolation=cv2.INTER_NEAREST)
                        center = mask_centroid(m_bin > 0.5)
                    else:
                        center = None

                    if center is None:
                        center = ((x1 + x2) // 2, (y1 + y2) // 2)

                    u, v = int(center[0]), int(center[1])
                    self._draw_crosshair(vis, u, v)

                    # 鍍忕礌鍧愭爣
                    self._put_outlined(vis, f"({u}, {v})",
                                       (u + 14, v - 14), 0.6, _WHITE)

                    # 娣卞害 + 鐩告満鍧愭爣
                    if self.fx is not None:
                        z = sample_center_depth(
                            depth, u, v, self.depth_scale,
                            window=self.depth_window,
                            min_m=self.min_depth, max_m=self.max_depth)
                        if z is not None:
                            in_range  = self.range_min <= z <= self.range_max
                            px, py, pz = deproject_pixel(
                                u, v, z,
                                self.fx, self.fy, self.cx, self.cy)
                            self._draw_range_badge(vis, u, v, in_range)
                            self._draw_cam_coords(vis, u, v, px, py, pz, h, w)

        # 鈹€鈹€ HUD锛堝乏涓婅锛夆攢鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
        self._draw_hud(vis)
        self._draw_det_count(vis, n_det)

        # 鈹€鈹€ 鍧愭爣绯诲浘渚嬶紙鍙充笅瑙掞級鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
        self._draw_legend(vis)

        # 鈹€鈹€ 鍙戝竷 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
        out_msg = self.bridge.cv2_to_imgmsg(vis, encoding="bgr8")
        out_msg.header = color_msg.header
        self.debug_pub.publish(out_msg)

    # 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲 缁樺浘宸ュ叿
    @staticmethod
    def _put_outlined(img: np.ndarray, text: str, pos: tuple,
                      scale: float, color: tuple) -> None:
        """甯﹂粦鑹叉弿杈圭殑鏂囧瓧锛堟彁楂樺彲璇绘€э級銆�"""
        cv2.putText(img, text, pos,
                    cv2.FONT_HERSHEY_SIMPLEX, scale, _BLACK, 3, cv2.LINE_AA)
        cv2.putText(img, text, pos,
                    cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)

    def _draw_crosshair(self, img: np.ndarray, u: int, v: int,
                        arm: int = 22, thick: int = 2) -> None:
        """绾㈣壊鍗佸瓧绾� + 涓績鐐� + 澶栧渾銆�"""
        cv2.line(img, (u - arm, v), (u + arm, v), _RED, thick, cv2.LINE_AA)
        cv2.line(img, (u, v - arm), (u, v + arm), _RED, thick, cv2.LINE_AA)
        cv2.circle(img, (u, v), 5,   _RED, -1, cv2.LINE_AA)
        cv2.circle(img, (u, v), arm, _RED,  1, cv2.LINE_AA)

    def _draw_range_badge(self, img: np.ndarray, u: int, v: int,
                          in_range: bool) -> None:
        """鐩爣鍙充晶鏄剧ず IN RANGE / OUT OF RANGE 寰界珷銆�"""
        text = "IN RANGE" if in_range else "OUT OF RANGE"
        col  = _GREEN if in_range else _YELLOW
        self._put_outlined(img, text, (u + 14, v + 20), 0.6, col)

    def _draw_cam_coords(self, img: np.ndarray,
                         u: int, v: int,
                         px: float, py: float, pz: float,
                         img_h: int, img_w: int) -> None:
        """鍦ㄧ洰鏍囬檮杩戠粯鍒朵笁缁村潗鏍囷紝鑷姩閬垮厤瓒呭嚭鍥惧儚杈圭晫銆�"""
        lines = [
            (f"X: {px:+.3f} m  ({'left'  if px < 0 else 'right'})", _CYAN),
            (f"Y: {py:+.3f} m  ({'up'    if py < 0 else 'down' })", _CYAN),
            (f"Z: {pz: .3f} m  (depth)",                             _CYAN),
        ]
        line_h = 22
        total_h = len(lines) * line_h
        ox = min(u + 14, img_w - 160)
        oy = v + 48
        # 瓒呭嚭搴曢儴鏃朵笂绉�
        if oy + total_h > img_h - 10:
            oy = v - total_h - 30
        for idx, (txt, col) in enumerate(lines):
            self._put_outlined(img, txt, (ox, oy + idx * line_h), 0.50, col)

    def _draw_hud(self, img: np.ndarray) -> None:
        """宸︿笂瑙� FPS銆�"""
        self._put_outlined(img, f"FPS: {self._fps:.1f}", (10, 32), 0.85, _WHITE)

    def _draw_det_count(self, img: np.ndarray, n: int) -> None:
        """宸︿笂瑙掓娴嬫暟閲忥紙FPS 涓嬫柟锛夈€�"""
        col = _GREEN if n > 0 else _GRAY
        self._put_outlined(img, f"Det: {n}", (10, 62), 0.75, col)

    def _draw_legend(self, img: np.ndarray) -> None:
        """鍙充笅瑙掔浉鏈哄潗鏍囩郴鍥句緥銆�"""
        h, w   = img.shape[:2]
        ox, oy = w - 145, h - 100

        # 鑳屾櫙妗�
        cv2.rectangle(img, (ox - 6, oy - 22), (w - 4, h - 4), _BLACK, -1)
        cv2.rectangle(img, (ox - 6, oy - 22), (w - 4, h - 4), _GRAY,   1)
        cv2.putText(img, "Camera Frame",
                    (ox, oy - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.40, _GRAY, 1, cv2.LINE_AA)

        # 鍘熺偣
        ex, ey = ox + 28, oy + 38
        cv2.circle(img, (ex, ey), 4, _WHITE, -1)

        # X 杞达紙鍙筹級
        cv2.arrowedLine(img, (ex, ey), (ex + 38, ey), _RED, 2, tipLength=0.28)
        cv2.putText(img, "X right", (ex + 42, ey + 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, _RED, 1, cv2.LINE_AA)

        # Y 杞达紙涓嬶級
        cv2.arrowedLine(img, (ex, ey), (ex, ey + 38), _GREEN, 2, tipLength=0.28)
        cv2.putText(img, "Y down", (ex - 8, ey + 54),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, _GREEN, 1, cv2.LINE_AA)

        # Z 杞达紙鍚戝墠锛岀敤鍦嗙偣绗﹀彿琛ㄧず杩涘叆灞忓箷鐨勬柟鍚戯級
        cv2.circle(img, (ex, ey), 11, _CYAN, 1)
        cv2.circle(img, (ex, ey),  2, _CYAN, -1)
        cv2.putText(img, "Z fwd", (ex - 60, ey + 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, _CYAN, 1, cv2.LINE_AA)


# 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲 鍏ュ彛
def main(args=None) -> None:
    rclpy.init(args=args)
    node = DebugVisNode()
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

