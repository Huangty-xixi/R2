# -*- coding: utf-8 -*-
"""
Camera detector node (detector)
================================
Responsibility: Use YOLOv8 segmentation model to identify target objects,
extract depth from the centroid of the target segmentation mask,
back-project to camera frame, transform to gripper frame via hand-eye calibration,
and publish TargetDepth.

Reusable design:
- All topic names configured via parameters / launch remap, not hardcoded;
- Model path, camera intrinsics, depth scale, acceptable range, hand-eye matrix all from YAML params;
- Online detection enable/disable via SetDetection service (disabled = no inference, equivalent to camera on/off);
- Class filtering (target_class_ids empty = accept all);
- No dependency on any business state machine; simply "receive enable -> publish target center depth".
"""

from typing import List, Optional

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import message_filters
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo

from vision_grasp_interfaces.msg import TargetDepth
from vision_grasp_interfaces.srv import SetDetection

from .geometry_utils import (
    sample_center_depth,
    deproject_pixel,
    transform_point,
    mask_centroid,
    build_tf_matrix,
)

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None


class DetectorNode(Node):
    def __init__(self) -> None:
        super().__init__("camera_detector")

        # ---------------- Parameter declarations ----------------
        # Topics (remappable, for reusability)
        self.declare_parameter("color_topic", "/camera/color/image_raw")
        self.declare_parameter("depth_topic",
                               "/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")
        self.declare_parameter("target_topic", "/target_depth")

        # Model and inference
        self.declare_parameter("model_path", "")
        self.declare_parameter("device", "cpu")          # 'cpu' / 'cuda:0'
        self.declare_parameter("conf_threshold", 0.5)
        self.declare_parameter("iou_threshold", 0.45)
        self.declare_parameter("target_class_ids", [""])
        self.declare_parameter("imgsz", 640)

        # Depth
        self.declare_parameter("depth_scale", 0.001)      # D435i: mm->m
        self.declare_parameter("depth_window", 5)         # center neighborhood median window
        self.declare_parameter("min_depth_m", 0.05)
        self.declare_parameter("max_depth_m", 5.0)

        # Acceptable grasping range (target distance from camera) -- task working distance 0.5~1.0 m
        self.declare_parameter("range_min_m", 0.50)
        self.declare_parameter("range_max_m", 1.00)

        # Hand-eye calibration T_gripper_camera: translation + quaternion (preferred) or 3x3 rotation
        self.declare_parameter("hand_eye_translation", [0.0, 0.0, 0.0])
        self.declare_parameter("hand_eye_quaternion", [0.0, 0.0, 0.0, 1.0])
        self.declare_parameter("hand_eye_rotation_matrix", [0.0])  # 9 elements, optional

        # Whether detection is enabled on startup (generally false, wait for state machine)
        self.declare_parameter("enabled_on_start", False)

        # ---------------- Read parameters ----------------
        self.depth_scale = float(self.get_parameter("depth_scale").value)
        self.depth_window = int(self.get_parameter("depth_window").value)
        self.min_depth = float(self.get_parameter("min_depth_m").value)
        self.max_depth = float(self.get_parameter("max_depth_m").value)
        self.range_min = float(self.get_parameter("range_min_m").value)
        self.range_max = float(self.get_parameter("range_max_m").value)
        self.conf = float(self.get_parameter("conf_threshold").value)
        self.iou = float(self.get_parameter("iou_threshold").value)
        self.imgsz = int(self.get_parameter("imgsz").value)
        self.device = self.get_parameter("device").value
        raw = self.get_parameter("target_class_ids").value
        self.target_class_names = [s for s in (raw or []) if s]   # raw=None 时锟斤拷同锟节匡拷锟叫憋拷锟斤拷锟斤拷锟斤拷全锟斤拷锟斤拷锟?

        self._enabled = bool(self.get_parameter("enabled_on_start").value)

        # Hand-eye matrix
        self.T_gripper_camera = self._load_hand_eye()
        self.get_logger().info(f"Hand-eye calibration matrix T_gripper_camera=\n{self.T_gripper_camera}")

        # ---------------- Model loading ----------------
        self.bridge = CvBridge()
        self.model = None
        self._load_model()

        # Camera intrinsics (populated from camera_info)
        self.fx = self.fy = self.cx = self.cy = None

        # ---------------- ROS interface ----------------
        self.target_pub = self.create_publisher(
            TargetDepth, self.get_parameter("target_topic").value, 10)

        self.create_subscription(
            CameraInfo, self.get_parameter("camera_info_topic").value,
            self._on_camera_info, 10)

        sensor_qos = QoSProfile(depth=5,
                                reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST)
        color_sub = message_filters.Subscriber(
            self, Image, self.get_parameter("color_topic").value, qos_profile=sensor_qos)
        depth_sub = message_filters.Subscriber(
            self, Image, self.get_parameter("depth_topic").value, qos_profile=sensor_qos)
        # Approximate time synchronization to align color/depth
        self._sync = message_filters.ApproximateTimeSynchronizer(
            [color_sub, depth_sub], queue_size=5, slop=0.05)
        self._sync.registerCallback(self._on_frame)

        # Online detection on/off service
        self.create_service(SetDetection, "~/set_detection", self._on_set_detection)

        self.get_logger().info(
            f"camera_detector started, enabled={self._enabled}, "
            f"acceptable range=[{self.range_min}, {self.range_max}] m")

    # ------------------------------------------------------------ Initialization helpers
    def _load_hand_eye(self) -> np.ndarray:
        t = list(self.get_parameter("hand_eye_translation").value)
        try:
            rot = list(self.get_parameter("hand_eye_rotation_matrix").value or [])
        except Exception:
            rot = []
        quat = list(self.get_parameter("hand_eye_quaternion").value)
        if len(rot) == 9:
            return build_tf_matrix(t, rotation_matrix=rot)
        return build_tf_matrix(t, quaternion=quat)

    def _load_model(self) -> None:
        path = self.get_parameter("model_path").value
        if not path:
            self.get_logger().warn("model_path not configured, detection will be skipped")
            return
        if YOLO is None:
            self.get_logger().error("ultralytics not installed, cannot load YOLOv8 (pip install ultralytics)")
            return
        try:
            self.model = YOLO(path)
            self.get_logger().info(f"Loaded YOLOv8 segmentation model: {path}")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Failed to load model: {exc}")
            self.model = None

    # ------------------------------------------------------------ Callbacks
    def _on_camera_info(self, msg: CameraInfo) -> None:
        # K = [fx 0 cx; 0 fy cy; 0 0 1]
        self.fx, self.fy = msg.k[0], msg.k[4]
        self.cx, self.cy = msg.k[2], msg.k[5]

    def _on_set_detection(self, req: SetDetection.Request,
                          resp: SetDetection.Response) -> SetDetection.Response:
        self._enabled = bool(req.enable)
        resp.success = True
        resp.message = f"detection {'enabled' if self._enabled else 'disabled'}"
        self.get_logger().info(f"Detection state switched to: {self._enabled}")
        return resp

    def _on_frame(self, color_msg: Image, depth_msg: Image) -> None:
        if not self._enabled or self.model is None:
            return
        if self.fx is None:
            self.get_logger().warn("camera_info not yet received, skipping frame", throttle_duration_sec=2.0)
            return

        color = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")

        best = self._infer_best_target(color)
        if best is None:
            return
        class_id, class_name, conf, mask = best

        # ---- Get target "center" pixel (mask centroid) ----
        center = mask_centroid(mask)
        if center is None:
            return
        u, v = center

        # ---- Target center depth (robust median in center neighborhood) ----
        z = sample_center_depth(depth, u, v, self.depth_scale,
                                window=self.depth_window,
                                min_m=self.min_depth, max_m=self.max_depth)
        if z is None:
            self.get_logger().warn("No valid depth at target center, dropping frame",
                                   throttle_duration_sec=1.0)
            return

        # ---- Back-projection + hand-eye transform ----
        p_cam = deproject_pixel(u, v, z, self.fx, self.fy, self.cx, self.cy)
        p_grip = transform_point(self.T_gripper_camera, p_cam)
        in_range = self.range_min <= z <= self.range_max

        # ---- Publish ----
        out = TargetDepth()
        out.header = color_msg.header
        out.class_name = class_name
        out.class_id = int(class_id)
        out.confidence = float(conf)
        out.center_u = int(u)
        out.center_v = int(v)
        out.center_depth = float(z)
        out.position_camera.x, out.position_camera.y, out.position_camera.z = p_cam
        out.position_gripper.x, out.position_gripper.y, out.position_gripper.z = p_grip
        out.valid = True
        out.in_range = bool(in_range)
        self.target_pub.publish(out)

    # ------------------------------------------------------------ Inference
    def _infer_best_target(self, color: np.ndarray):
        """Return the highest-confidence target (class_id, class_name, conf, mask) or None."""
        results = self.model.predict(
            source=color, conf=self.conf, iou=self.iou,
            imgsz=self.imgsz, device=self.device, verbose=False)
        if not results:
            return None
        res = results[0]
        if res.masks is None or res.boxes is None or len(res.boxes) == 0:
            return None

        names = res.names
        h, w = color.shape[:2]
        best = None
        best_conf = -1.0
        masks = res.masks.data.cpu().numpy()          # (n, mh, mw)
        cls_arr = res.boxes.cls.cpu().numpy().astype(int)
        conf_arr = res.boxes.conf.cpu().numpy()

        for i in range(len(cls_arr)):
            cid = int(cls_arr[i])
            cname = names.get(cid, str(cid))
            if self.target_class_names and cname not in self.target_class_names:
                continue
            c = float(conf_arr[i])
            if c <= best_conf:
                continue
            m = masks[i]
            if m.shape != (h, w):  # resize mask to original image size
                import cv2
                m = cv2.resize(m, (w, h), interpolation=cv2.INTER_NEAREST)
            best = (cid, cname, c, (m > 0.5))
            best_conf = c
        return best


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DetectorNode()
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
