# -*- coding: utf-8 -*-
"""
debug_vis_node.py  ——  检测可视化调试节点
==========================================
独立运行 YOLOv8 推理（与 detector_node 完全解耦，仅用于可视化），
将标注后的图像发布到 /debug_image，可在 rqt_image_view 中实时查看。

可视化内容：
    · 所有目标的 YOLO 包围框（绿=最优目标，蓝=其他目标）
    · 分割掩膜轮廓
    · 最优目标中心十字线 + 中心像素坐标 (u, v)
    · 最优目标相机坐标 (x, y, z) 单位 m
    · IN RANGE / OUT OF RANGE 状态提示
    · 实时推理帧率 FPS
    · 右下角相机坐标系图例

相机坐标系（ROS optical frame，与 deproject_pixel 一致）：
    X →  向右（u 增大方向）  正 = 目标在相机右侧，负 = 左侧
    Y ↓  向下（v 增大方向）  正 = 目标在相机下方，负 = 上方
    Z ●  向前（光轴/深度）   正 = 目标在相机前方（始终正值）

使用方法：
    # 把本文件放到 vision_grasp_camera/vision_grasp_camera/ 下，
    # 在 setup.py 的 console_scripts 里加一行：
    #     'debug_vis = vision_grasp_camera.debug_vis_node:main',
    # 然后重新 colcon build，再执行：
    ros2 run vision_grasp_camera debug_vis \
      --ros-args --params-file ~/RC2026/src/vision_grasp_bringup/config/camera_params.yaml

    # 在 rqt 中查看：
    rqt
    # Plugins → Visualization → Image View → 选择 /debug_image
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

# ─── 颜色常量 (BGR) ──────────────────────────────────────────────────────────
_GREEN  = (50,  210,  50)    # 最优目标框
_BLUE   = (200, 120,  30)    # 其他目标框
_RED    = (30,   30, 220)    # 十字线
_CYAN   = (220, 200,   0)    # 坐标文字
_YELLOW = (0,   200, 220)    # OUT OF RANGE
_WHITE  = (255, 255, 255)
_BLACK  = (0,     0,   0)
_GRAY   = (160, 160, 160)


class DebugVisNode(Node):
    def __init__(self) -> None:
        super().__init__("debug_vis")

        # ── 参数声明（与 camera_params.yaml 中 camera_detector 段同名，可共用） ──
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
        self.declare_parameter("target_class_ids",   [""])   # [""] = 接受全部类别
        self.declare_parameter("depth_scale",        0.001)
        self.declare_parameter("depth_window",       5)
        self.declare_parameter("min_depth_m",        0.05)
        self.declare_parameter("max_depth_m",        5.0)
        self.declare_parameter("range_min_m",        0.50)
        self.declare_parameter("range_max_m",        1.00)

        # ── 读取参数 ──────────────────────────────────────────────────────────
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
        self.target_class_names = [s for s in (raw or []) if s]   # 空列表=接受全部

        # ── 内部状态 ──────────────────────────────────────────────────────────
        self.fx = self.fy = self.cx = self.cy = None
        self.bridge   = CvBridge()
        self._last_t  = time.monotonic()
        self._fps     = 0.0

        # ── 模型加载 ──────────────────────────────────────────────────────────
        self.model = None
        self._load_model()

        # ── 发布者 ────────────────────────────────────────────────────────────
        self.debug_pub = self.create_publisher(
            Image, self.get_parameter("debug_image_topic").value, 5)

        # ── 订阅者 ────────────────────────────────────────────────────────────
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
            f"debug_vis 启动  "
            f"model={self.get_parameter('model_path').value}  "
            f"filter={self.target_class_names or '全部类别'}  "
            f"range=[{self.range_min}, {self.range_max}] m")

    # ════════════════════════════════════════════════════════ 初始化
    def _load_model(self) -> None:
        path = self.get_parameter("model_path").value
        if not path:
            self.get_logger().warn("model_path 未配置，跳过模型加载")
            return
        if YOLO is None:
            self.get_logger().error("ultralytics 未安装（pip install ultralytics）")
            return
        try:
            self.model = YOLO(path)
            self.get_logger().info(f"模型已加载：{path}")
        except Exception as exc:                    # noqa: BLE001
            self.get_logger().error(f"模型加载失败：{exc}")

    # ════════════════════════════════════════════════════════ 回调
    def _on_camera_info(self, msg: CameraInfo) -> None:
        """从 camera_info 读取内参，用于反投影计算相机坐标。"""
        self.fx, self.fy = msg.k[0], msg.k[4]
        self.cx, self.cy = msg.k[2], msg.k[5]

    def _on_frame(self, color_msg: Image, depth_msg: Image) -> None:
        """主回调：推理 → 标注 → 发布。"""
        if self.model is None:
            return

        # FPS（帧间时间）
        now = time.monotonic()
        dt  = now - self._last_t
        self._last_t = now
        self._fps = 1.0 / dt if dt > 1e-6 else 0.0

        color = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        h, w  = color.shape[:2]
        vis   = color.copy()

        # ── YOLO 推理 ────────────────────────────────────────────────────────
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

            # 先找最优目标（置信度最高且通过类别过滤）
            for i in range(len(cls_arr)):
                cname = names.get(int(cls_arr[i]), str(cls_arr[i]))
                if self.target_class_names and cname not in self.target_class_names:
                    continue
                n_det += 1
                if float(conf_arr[i]) > best_conf:
                    best_conf = float(conf_arr[i])
                    best_idx  = i

            # 绘制所有通过过滤的目标
            for i in range(len(cls_arr)):
                cid   = int(cls_arr[i])
                cname = names.get(cid, str(cid))
                if self.target_class_names and cname not in self.target_class_names:
                    continue

                x1, y1, x2, y2 = xyxy_arr[i]
                is_best = (i == best_idx)
                col     = _GREEN if is_best else _BLUE
                thick   = 3 if is_best else 1

                # 掩膜轮廓
                if masks_np is not None:
                    m = masks_np[i]
                    if m.shape != (h, w):
                        m = cv2.resize(m.astype(np.float32), (w, h),
                                       interpolation=cv2.INTER_NEAREST)
                    contours, _ = cv2.findContours(
                        (m > 0.5).astype(np.uint8),
                        cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    cv2.drawContours(vis, contours, -1, col, 2)

                # YOLO 包围框
                cv2.rectangle(vis, (x1, y1), (x2, y2), col, thick)

                # 标签背景 + 文字
                label = f"{cname}  {conf_arr[i]:.2f}"
                (tw, th), _ = cv2.getTextSize(
                    label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
                cv2.rectangle(vis, (x1, y1 - th - 8), (x1 + tw + 6, y1), col, -1)
                cv2.putText(vis, label, (x1 + 3, y1 - 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, _BLACK, 1, cv2.LINE_AA)

                # ── 最优目标：十字线 + 深度 + 坐标 ──────────────────────────
                if is_best:
                    # 掩膜质心（与 detector_node 完全一致）
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

                    # 像素坐标
                    self._put_outlined(vis, f"({u}, {v})",
                                       (u + 14, v - 14), 0.6, _WHITE)

                    # 深度 + 相机坐标
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

        # ── HUD（左上角）────────────────────────────────────────────────────
        self._draw_hud(vis)
        self._draw_det_count(vis, n_det)

        # ── 坐标系图例（右下角）─────────────────────────────────────────────
        self._draw_legend(vis)

        # ── 发布 ─────────────────────────────────────────────────────────────
        out_msg = self.bridge.cv2_to_imgmsg(vis, encoding="bgr8")
        out_msg.header = color_msg.header
        self.debug_pub.publish(out_msg)

    # ════════════════════════════════════════════════════════ 绘图工具
    @staticmethod
    def _put_outlined(img: np.ndarray, text: str, pos: tuple,
                      scale: float, color: tuple) -> None:
        """带黑色描边的文字（提高可读性）。"""
        cv2.putText(img, text, pos,
                    cv2.FONT_HERSHEY_SIMPLEX, scale, _BLACK, 3, cv2.LINE_AA)
        cv2.putText(img, text, pos,
                    cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)

    def _draw_crosshair(self, img: np.ndarray, u: int, v: int,
                        arm: int = 22, thick: int = 2) -> None:
        """红色十字线 + 中心点 + 外圆。"""
        cv2.line(img, (u - arm, v), (u + arm, v), _RED, thick, cv2.LINE_AA)
        cv2.line(img, (u, v - arm), (u, v + arm), _RED, thick, cv2.LINE_AA)
        cv2.circle(img, (u, v), 5,   _RED, -1, cv2.LINE_AA)
        cv2.circle(img, (u, v), arm, _RED,  1, cv2.LINE_AA)

    def _draw_range_badge(self, img: np.ndarray, u: int, v: int,
                          in_range: bool) -> None:
        """目标右侧显示 IN RANGE / OUT OF RANGE 徽章。"""
        text = "IN RANGE" if in_range else "OUT OF RANGE"
        col  = _GREEN if in_range else _YELLOW
        self._put_outlined(img, text, (u + 14, v + 20), 0.6, col)

    def _draw_cam_coords(self, img: np.ndarray,
                         u: int, v: int,
                         px: float, py: float, pz: float,
                         img_h: int, img_w: int) -> None:
        """在目标附近绘制三维坐标，自动避免超出图像边界。"""
        lines = [
            (f"X: {px:+.3f} m  ({'left'  if px < 0 else 'right'})", _CYAN),
            (f"Y: {py:+.3f} m  ({'up'    if py < 0 else 'down' })", _CYAN),
            (f"Z: {pz: .3f} m  (depth)",                             _CYAN),
        ]
        line_h = 22
        total_h = len(lines) * line_h
        ox = min(u + 14, img_w - 160)
        oy = v + 48
        # 超出底部时上移
        if oy + total_h > img_h - 10:
            oy = v - total_h - 30
        for idx, (txt, col) in enumerate(lines):
            self._put_outlined(img, txt, (ox, oy + idx * line_h), 0.50, col)

    def _draw_hud(self, img: np.ndarray) -> None:
        """左上角 FPS。"""
        self._put_outlined(img, f"FPS: {self._fps:.1f}", (10, 32), 0.85, _WHITE)

    def _draw_det_count(self, img: np.ndarray, n: int) -> None:
        """左上角检测数量（FPS 下方）。"""
        col = _GREEN if n > 0 else _GRAY
        self._put_outlined(img, f"Det: {n}", (10, 62), 0.75, col)

    def _draw_legend(self, img: np.ndarray) -> None:
        """右下角相机坐标系图例。"""
        h, w   = img.shape[:2]
        ox, oy = w - 145, h - 100

        # 背景框
        cv2.rectangle(img, (ox - 6, oy - 22), (w - 4, h - 4), _BLACK, -1)
        cv2.rectangle(img, (ox - 6, oy - 22), (w - 4, h - 4), _GRAY,   1)
        cv2.putText(img, "Camera Frame",
                    (ox, oy - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.40, _GRAY, 1, cv2.LINE_AA)

        # 原点
        ex, ey = ox + 28, oy + 38
        cv2.circle(img, (ex, ey), 4, _WHITE, -1)

        # X 轴（右）
        cv2.arrowedLine(img, (ex, ey), (ex + 38, ey), _RED, 2, tipLength=0.28)
        cv2.putText(img, "X right", (ex + 42, ey + 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, _RED, 1, cv2.LINE_AA)

        # Y 轴（下）
        cv2.arrowedLine(img, (ex, ey), (ex, ey + 38), _GREEN, 2, tipLength=0.28)
        cv2.putText(img, "Y down", (ex - 8, ey + 54),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, _GREEN, 1, cv2.LINE_AA)

        # Z 轴（向前，用圆点符号表示进入屏幕的方向）
        cv2.circle(img, (ex, ey), 11, _CYAN, 1)
        cv2.circle(img, (ex, ey),  2, _CYAN, -1)
        cv2.putText(img, "Z fwd", (ex - 60, ey + 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, _CYAN, 1, cv2.LINE_AA)


# ════════════════════════════════════════════════════════════ 入口
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

