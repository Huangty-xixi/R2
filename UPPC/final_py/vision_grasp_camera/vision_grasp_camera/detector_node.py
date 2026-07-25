# -*- coding: utf-8 -*-
"""
detector_node.py —— 相机检测节点（把摄像头画面转成目标 3D 坐标）

数据流（= FreeRTOS 里一个任务的数据处理链）：
  摄像头彩色图 + 深度图（话题同步后到达）
    → YOLOv8 分割推理（找到目标在哪，生成掩膜）
    → 掩膜质心 = 目标的像素坐标 (u, v)
    → 质心邻域中值滤波 = 鲁棒的深度 Z（滤掉空洞和噪声）
    → 针孔反投影 = 像素 + 深度 → 相机系 3D 坐标 (Xc, Yc, Zc)
    → 手眼标定矩阵 T_gripper_camera × (Xc,Yc,Zc) = 夹爪系 3D 坐标
    → 发布 TargetDepth 消息到 /target_depth 话题
    → serial_bridge 收到 → 打包串口帧 → STM32 用来做对准

所有话题名、模型路径、相机内参、深度参数全从 YAML 配置读取，不硬编码。
"""

# ── import 语句 = C 的 #include ──
# 每个 import 都是"我要用别人写的代码"，和 C 的 #include "xxx.h" 一样
from typing import List, Optional     # 类型注解（= 文档用，不影响运行，帮你和 IDE 知道变量是什么类型）

import numpy as np                   # Python 科学计算库。np.ndarray = 多维数组（= C 的 float buf[640][480] 但功能强一万倍）
                                      # 核心操作：np.array([1,2,3]) 创建数组、arr[0:10] 切片、arr.mean() 均值

import rclpy                          # Python 版 ROS2 客户端库（= FreeRTOS 内核 + HAL 库的总和）
from rclpy.node import Node          # ROS2 节点基类——所有节点的爹。Node 自带 create_publisher / create_subscription / create_timer
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy  # QoS = Quality of Service = 话题"服务质量"
                                                                      # 类比 C 的 CAN 帧优先级：你是要"保证送达"还是"丢了就算了"？

import message_filters               # ROS2 的消息同步器——"同时收到彩色图和深度图才干活"
                                      # 类比 C: 等两个 CAN ID 的报文都到了才处理
from cv_bridge import CvBridge       # OpenCV 格式 ↔ ROS Image 消息格式 互转
                                      # 类比 C: 把 uint8_t buf[] 转成你的结构体类型
from sensor_msgs.msg import Image, CameraInfo  # ROS 标准消息：图像帧 / 相机内参矩阵
                                                # Image = 一帧图像（含时间戳、宽高、编码格式、像素数据）
                                                # CameraInfo = 相机标定参数（含内参矩阵 K）

from vision_grasp_interfaces.msg import TargetDepth  # 你们自己定义的消息：检测到目标的 3D 坐标
from vision_grasp_interfaces.srv import SetDetection # 你们自己定义的服务：开关摄像头检测

# from .xxx import ... = 从同目录下的其他 .py 文件引入函数
# "." 表示"当前包"（= C 里 #include "./geometry_utils.h" 的 ./）
from .geometry_utils import (
    sample_center_depth,    # 在目标中心像素周围取一个小区域的中值深度（= 中值滤波，抗噪声）
    deproject_pixel,        # 针孔相机反投影：像素坐标(u,v) + 深度Z → 相机系3D坐标(X,Y,Z)
    transform_point,        # 4×4 齐次变换矩阵 × (X,Y,Z) = 坐标系变换（= C 的矩阵乘法）
    mask_centroid,          # 求二值掩膜（黑白图）的质量中心——所有白色像素的平均坐标
    build_tf_matrix,        # 把平移+旋转拼成 4×4 齐次变换矩阵（= C 里手写矩阵赋值）
)

# ── 尝试导入 YOLOv8 库（pip install ultralytics）──
# 如果电脑上没装这个库 → YOLO = None → 检测跳过但不崩溃
try:                               # try = "尝试执行，如果出错跳到 except"
    from ultralytics import YOLO   # YOLOv8 分割模型的主类
except ImportError:                # ImportError = "import 失败"（库没装 / 拼写错误）
    YOLO = None                    # 设成 None（= C 的 NULL），后面代码会检查


class DetectorNode(Node):          # 继承 Node = 定义一个 ROS2 节点（= FreeRTOS 任务）
    def __init__(self) -> None:    # __init__ = 构造函数，创建节点对象时自动调一次（= 任务初始化函数）

        # ── 第一步：调父类 Node 的初始化（注册节点到 ROS2 框架）──
        # super() = 父类 Node。super().__init__("名字") = 调 Node.__init__("名字")
        # 等价于 C: baseModule_Init(&me->base, "camera_detector")
        super().__init__("camera_detector")   # 节点名 = "camera_detector"，终端 ros2 node list 能看到

        # ═══════════════════════════════════════════════════════════
        # 第二步：声明可配参数（= C 里 #define XXX 默认值，但运行时能被命令行/YAML 覆盖）
        # declare_parameter("参数名", 默认值) 是 Node 自带的方法
        # 这个节点所有对外接口——话题名、模型路径、深度阈值——全是参数化的，不写死
        # ═══════════════════════════════════════════════════════════

        # ——— 话题名（可以重映射，适应不同的相机驱动命名）———
        self.declare_parameter("color_topic", "/camera/color/image_raw")         # 彩色图话题
        self.declare_parameter("depth_topic", "/camera/aligned_depth_to_color/image_raw")  # 深度图（已对齐到彩色图坐标）
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info") # 相机内参话题（含 K 矩阵）
        self.declare_parameter("target_topic", "/target_depth")                 # 检测结果发布到哪个话题

        # ——— YOLO 推理参数 ———
        self.declare_parameter("model_path", "")            # YOLO 模型权重文件路径（空=不加载，纯离线不给检测）
        self.declare_parameter("device", "cpu")             # 推理设备：'cpu'（慢但不用显卡）或 'cuda:0'（NVIDIA GPU）
        self.declare_parameter("conf_threshold", 0.5)       # 置信度阈值：低于此值的检测框直接丢弃（0.5=50%把握）
        self.declare_parameter("iou_threshold", 0.45)       # IoU 非极大抑制阈值：两个框重叠 >45% → 合并为一个
        self.declare_parameter("target_class_ids", [""])    # 要检测的类别名列表（空=全接受；["甲骨1","甲骨2"]=只检测这两类）
        self.declare_parameter("imgsz", 640)                 # YOLO 输入图像尺寸（像素），内部自动 resize

        # ——— 深度参数 ———
        self.declare_parameter("depth_scale", 0.001)        # 深度图的物理比例：D435i 原始值是毫米，×0.001 转成米
        self.declare_parameter("depth_window", 5)           # 中值滤波的窗口半径（像素）：在 5×5 邻域内取中值
        self.declare_parameter("min_depth_m", 0.05)         # 有效深度下限（米）：小于此值视为相机噪声/坏点
        self.declare_parameter("max_depth_m", 5.0)          # 有效深度上限（米）：大于此值视为无效（背景太远）

        # ——— 可抓取范围（抓取机械臂的工作距离）———
        self.declare_parameter("range_min_m", 0.50)  # 最短抓取距离（0.5米 = 太近了抓不到）
        self.declare_parameter("range_max_m", 1.00)  # 最长抓取距离（1.0米 = 太远了够不着）

        # ——— 手眼标定：相机 → 夹爪的坐标变换（eye-in-hand = 相机装在机械臂上）———
        self.declare_parameter("hand_eye_translation", [0.0, 0.0, 0.0])       # 平移向量 [x, y, z]（米）
        self.declare_parameter("hand_eye_quaternion", [0.0, 0.0, 0.0, 1.0])  # 四元数 [x, y, z, w]（单位四元数 = 不旋转）
        self.declare_parameter("hand_eye_rotation_matrix", [0.0])             # 3×3 旋转矩阵（9 个元素，和四元数二选一）

        # ——— 启动行为 ———
        self.declare_parameter("enabled_on_start", False)  # 启动时是否立刻开始检测？（False=等状态机下命令再开）

        # ═══════════════════════════════════════════════════════════
        # 第三步：把参数值读到成员变量里（self.xxx = 成员变量，相当于 C 结构体的字段）
        # self.get_parameter("名字").value → 取出参数的实际值（手动改过的 / 默认值）
        # float() / int() / bool() 是 Python 类型转换，和 C 的 (float)x 一样
        # ═══════════════════════════════════════════════════════════

        self.depth_scale = float(self.get_parameter("depth_scale").value)       # 深度比例
        self.depth_window = int(self.get_parameter("depth_window").value)       # 中值窗口
        self.min_depth = float(self.get_parameter("min_depth_m").value)         # 有效深度下限
        self.max_depth = float(self.get_parameter("max_depth_m").value)         # 有效深度上限
        self.range_min = float(self.get_parameter("range_min_m").value)         # 抓取范围下限
        self.range_max = float(self.get_parameter("range_max_m").value)         # 抓取范围上限
        self.conf = float(self.get_parameter("conf_threshold").value)           # YOLO 置信度
        self.iou = float(self.get_parameter("iou_threshold").value)             # YOLO IoU
        self.imgsz = int(self.get_parameter("imgsz").value)                     # YOLO 输入尺寸
        self.device = self.get_parameter("device").value                        # CPU/GPU（字符串，不用转类型）
        raw = self.get_parameter("target_class_ids").value                      # 读取原始类别名列表
        # 下面这行是 Python 的"列表推导式"：[表达式 for 变量 in 列表 if 条件]
        # [s for s in (raw or []) if s] 拆开：
        #   (raw or []) = 如果 raw 是 None 就用空列表 []（Python 的安全取法）
        #   for s in ...  = 遍历每个元素
        #   if s           = 只保留非空字符串（空字符串 "" = False）
        # 等效 C：char* result[SIZE]; int j=0; for(i=0;i<SIZE;i++){if(strlen(raw[i]))result[j++]=raw[i];}
        self.target_class_names = [s for s in (raw or []) if s]

        self._enabled = bool(self.get_parameter("enabled_on_start").value)      # 是否自动开检测

        # ——— 加载手眼标定矩阵（4×4 np.ndarray，对角线=1 其余=0 的单位矩阵）———
        self.T_gripper_camera = self._load_hand_eye()
        self.get_logger().info(f"Hand-eye calibration matrix T_gripper_camera=\n{self.T_gripper_camera}")

        # ——— 模型加载 ———
        self.bridge = CvBridge()     # CvBridge：ROS 的 Image 消息 ↔ OpenCV 的 Mat 格式互转器
                                      # 类比 C：一个工具函数，把 uint8_t buf[] 解析成 640×480 的像素矩阵
        self.model = None            # YOLO 模型对象，初始为 None（= C 的 NULL）。后面的 _load_model() 会填充
        self._load_model()

        # ——— 相机内参（等收到第一个 CameraInfo 消息后才赋值）———
        # Python 链式赋值：a = b = c = None → 四个变量全设为 None
        self.fx = self.fy = self.cx = self.cy = None  # fx/fy=焦距(像素), cx/cy=光心(像素)

        # ═══════════════════════════════════════════════════════════
        # 第四步：ROS2 通信——声明"我要发布/订阅/提供服务"
        # ═══════════════════════════════════════════════════════════

        # ——— 发布者（= C 里 CAN_Send 的声明）：检测到目标后发布 TargetDepth 消息 ———
        self.target_pub = self.create_publisher(
            TargetDepth,                                   # 消息类型（自定义的）
            self.get_parameter("target_topic").value,      # 话题名（从参数读，默认 "/target_depth"）
            10)                                            # 队列深度：积压最多 10 帧

        # ——— 订阅者 1（= C 里 CAN 接收中断的声明）：收相机内参 ———
        # 相机内参只需要收一次（镜头不变），不用高频，所以直接用 create_subscription 不用 message_filters
        self.create_subscription(
            CameraInfo,                                    # 消息类型
            self.get_parameter("camera_info_topic").value,  # 话题名
            self._on_camera_info,                           # 回调函数（收到消息时自动调）
            10)                                             # 队列深度

        # ——— 订阅者 2+3（= 两个 CAN 中断，但要"同时触发"才干活）：收彩色图 + 深度图 ———
        # 为什么要用 message_filters？因为彩色图和深度图是独立的话题，到达时间略有不同
        # 如果分开收：可能彩色帧 N 配了深度帧 N-1 的数据（时间没对准）
        # message_filters.ApproximateTimeSynchronizer 等两个话题的时间戳差 < 0.05 秒才调回调

        # QoS 配置：里程计/图像这种高频大数据用 BEST_EFFORT（丢了不重传，只保留最新帧）
        # depth=5 缓存最多 5 帧；KEEP_LAST 满了就丢最老的；BEST_EFFORT = 不保证送达（省带宽省 CPU）
        sensor_qos = QoSProfile(depth=5,
                                reliability=ReliabilityPolicy.BEST_EFFORT,  # 丢了就算了，不重传
                                history=HistoryPolicy.KEEP_LAST)            # 只保留最新的帧

        # message_filters.Subscriber = 特殊的订阅者，能被 ApproximateTimeSynchronizer 管理
        # 和 create_subscription 功能一样，但多了一个能力：可以被同步器同时等待
        color_sub = message_filters.Subscriber(          # 彩色图订阅者
            self, Image,                                  # 节点自身, 消息类型
            self.get_parameter("color_topic").value,      # 话题名
            qos_profile=sensor_qos)                       # QoS 传入
        depth_sub = message_filters.Subscriber(          # 深度图订阅者
            self, Image,
            self.get_parameter("depth_topic").value,
            qos_profile=sensor_qos)

        # ApproximateTimeSynchronizer = "近似时间同步器"
        # 参数：[color_sub, depth_sub] = 要同步的两个订阅者
        # queue_size=5 = 最多缓存 5 对未处理的帧
        # slop=0.05 = 两帧时间戳相差 < 0.05 秒 就算"同时到达"
        # 类比 C: 两个 DMA 双缓冲都收完了 → 才调回调处理
        self._sync = message_filters.ApproximateTimeSynchronizer(
            [color_sub, depth_sub], queue_size=5, slop=0.05)
        # registerCallback 注册回调：两帧对齐成功时调 _on_frame(color_msg, depth_msg)
        self._sync.registerCallback(self._on_frame)

        # ——— 服务端（= C 里定义一个"别人可以调我"的函数）：状态机通过这个服务开关检测 ———
        # 和 create_publisher 不同——publisher 是"我往总线上广播"，
        # create_service 是"别人发请求给我，我处理完回答他"（= RPC 调用）
        self.create_service(SetDetection, "~/set_detection", self._on_set_detection)

        # ——— 启动日志 ———
        # f-string（f"xxx{变量}"）= C 的 printf("camera_detector started, enabled=%d", self._enabled)
        self.get_logger().info(
            f"camera_detector started, enabled={self._enabled}, "
            f"acceptable range=[{self.range_min}, {self.range_max}] m")

    # ═══════════════════════════════════════════════════════════
    # 初始化辅助函数（只在 __init__ 里被调一次，不参与运行时循环）
    # ═══════════════════════════════════════════════════════════

    def _load_hand_eye(self) -> np.ndarray:
        """
        把手眼标定参数（平移+旋转）拼成一个 4×4 齐次变换矩阵
        返回值类型 np.ndarray = numpy 的 4×4 二维数组

        优先级：旋转矩阵（9 元素）> 四元数 > 单位矩阵（不旋转）
        """
        # list() 把 ROS2 参数值（可能是 tuple/list/array）统一转成 Python 列表
        t = list(self.get_parameter("hand_eye_translation").value)     # 平移 [x,y,z]
        try:                                                           # try = "尝试，出错跳到 except"
            rot = list(self.get_parameter("hand_eye_rotation_matrix").value or [])
        except Exception:                                              # 任何错误（比如参数类型不对）
            rot = []                                                   # 默认为空列表
        quat = list(self.get_parameter("hand_eye_quaternion").value)   # 四元数 [x,y,z,w]

        # Python 的 len() = 数组长度（= C 的 sizeof(arr)/sizeof(arr[0]) 或者 strlen）
        if len(rot) == 9:                                              # 提供了 3×3 旋转矩阵
            return build_tf_matrix(t, rotation_matrix=rot)              # 用旋转矩阵构造
        return build_tf_matrix(t, quaternion=quat)                      # 用四元数构造

    def _load_model(self) -> None:
        """加载 YOLOv8 分割模型。路径为空 / 库没装 → 跳过不崩溃"""
        path = self.get_parameter("model_path").value
        # Python 的空/None 判断：空字符串 ""、None、0 在 if 里都是 False
        if not path:                                         # path 为空字符串 → 跳过
            self.get_logger().warn("model_path not configured, detection will be skipped")
            return                                           # return 后函数结束，不往下执行
        if YOLO is None:                                     # ultralytics 库没装
            self.get_logger().error("ultralytics not installed, cannot load YOLOv8 (pip install ultralytics)")
            return
        try:
            self.model = YOLO(path)                          # 加载模型文件（= C 里 fopen 读 .pt 权重文件）
            self.get_logger().info(f"Loaded YOLOv8 segmentation model: {path}")
        except Exception as exc:                              # exc = 异常对象（含错误描述）
            self.get_logger().error(f"Failed to load model: {exc}")
            self.model = None                                 # 加载失败 → 清空，后面检测跳过

    # ═══════════════════════════════════════════════════════════
    # ROS2 回调函数（由 ROS2 框架自动调用，你不需要写 while(1) 轮询）
    # ═══════════════════════════════════════════════════════════

    def _on_camera_info(self, msg: CameraInfo) -> None:
        """收到相机内参（只需一次，后续帧复用）"""
        # msg.k 是 9 元素的 float64 数组（行优先存储）
        # 相机内参矩阵 K = [[fx, 0, cx], [0, fy, cy], [0, 0, 1]]
        # 存储顺序：[fx, 0, cx, 0, fy, cy, 0, 0, 1]
        # 索引 0=fx, 2=cx, 4=fy, 5=cy
        self.fx, self.fy = msg.k[0], msg.k[4]    # 焦距（像素）——越大=视野越窄
        self.cx, self.cy = msg.k[2], msg.k[5]    # 光心（像素）——理论上是图像中心

    def _on_set_detection(self, req: SetDetection.Request,
                          resp: SetDetection.Response) -> SetDetection.Response:
        """
        服务回调：状态机通过 SetDetection 服务调这个函数 → 开关检测
        req = 请求对象（含 enable: bool）
        resp = 响应对象（要填 success 和 message 后返回）
        ROS2 协定：服务回调必须返回响应对象
        """
        self._enabled = bool(req.enable)                           # 开/关检测
        resp.success = True                                        # 操作成功
        # Python 的三元表达式：a if 条件 else b  ← 等效 C 的 条件 ? a : b
        resp.message = f"detection {'enabled' if self._enabled else 'disabled'}"
        self.get_logger().info(f"Detection state switched to: {self._enabled}")
        return resp                                                # 返回响应给调用方

    def _on_frame(self, color_msg: Image, depth_msg: Image) -> None:
        """
        ★ 核心处理函数 ★
        两帧时间同步后调这个函数（color 和 depth 的时间戳差 < 0.05s）

        数据流（每一步对应一个函数调用）：
          ROS Image → OpenCV Mat → YOLO 推理 → 掩膜质心 → 中值深度 → 反投影 → 手眼变换 → publish
        """
        # ── 前置检查：不满足条件直接跳过 ──
        # if not self._enabled: 检测关了 → 不干活
        # self.model is None: 模型没加载 → 不干活
        # Python 的 or：短路求值——如果第一个条件是 True，后面的不判断
        if not self._enabled or self.model is None:
            return

        # 相机内参还没收到（还没订阅到 CameraInfo 消息）
        if self.fx is None:
            # throttle_duration_sec=2.0：这条 warn 每 2 秒最多打一次（防止刷屏）
            self.get_logger().warn("camera_info not yet received, skipping frame",
                                   throttle_duration_sec=2.0)
            return

        # ── 第 1 步：ROS Image 消息 → OpenCV 格式（numpy 数组） ──
        # CvBridge.imgmsg_to_cv2 = 把 ROS 的 Image 消息解析成 OpenCV 能处理的像素矩阵
        # desired_encoding="bgr8" = 要 BGR 三通道彩色（OpenCV 默认是 BGR 不是 RGB）
        # desired_encoding="passthrough" = 保持原格式不变（深度图是 uint16 单通道）
        color = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")        # → numpy.ndarray (H, W, 3)
        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")  # → numpy.ndarray (H, W)

        # ── 第 2 步：YOLO 推理 → 找置信度最高的目标 ──
        # _infer_best_target 返回 (class_id, class_name, conf, mask) 或 None
        best = self._infer_best_target(color)
        if best is None:                                    # 没检测到任何目标
            return
        # Python 元组解包：一次性把 4 个返回值赋给 4 个变量（等效 C 里通过指针参数返回多个值）
        class_id, class_name, conf, mask = best              # class_id=int, class_name=str, conf=float, mask=bool数组

        # ── 第 3 步：掩膜质心（所有目标像素的平均位置） ──
        # mask 是一个 True/False 的二维数组（shape = 图像高宽），True = 目标像素
        # mask_centroid 遍历所有 True 像素，取行列坐标的平均值 → (u, v)
        center = mask_centroid(mask)
        if center is None:                                  # 掩膜是空的
            return
        u, v = center                                       # u = 列坐标(水平), v = 行坐标(垂直)

        # ── 第 4 步：在质心邻域取鲁棒的深度值 ──
        # 为什么不能直接读 depth[v][u]？因为深度图有空洞（反射不良）和噪声
        # sample_center_depth 在 (u,v) 周围取 window×window 的区域，求中值（不是均值）
        # 中值比均值更能抗噪声——一个坏点不会拉偏结果
        z = sample_center_depth(depth, u, v, self.depth_scale,
                                window=self.depth_window,          # 邻域半径
                                min_m=self.min_depth, max_m=self.max_depth)  # 有效深度范围
        if z is None:                                        # 邻域内没有有效深度
            self.get_logger().warn("No valid depth at target center, dropping frame",
                                   throttle_duration_sec=1.0)
            return

        # ── 第 5 步：像素 → 3D 坐标 ──
        # deproject_pixel：针孔相机反投影公式
        #   X = (u - cx) * Z / fx      # 水平实际距离
        #   Y = (v - cy) * Z / fy      # 垂直实际距离
        #   Z = Z                       # 深度即 Z
        p_cam = deproject_pixel(u, v, z, self.fx, self.fy, self.cx, self.cy)  # 相机坐标系下的 3D 点

        # transform_point：T_gripper_camera @ [Xc, Yc, Zc, 1]^T → 夹爪坐标系
        # @ = Python 的矩阵乘法运算符（numpy 里 ≡ C 的矩阵乘）
        p_grip = transform_point(self.T_gripper_camera, p_cam)    # 夹爪坐标系下的 3D 点

        # Python 的链式比较：a <= b <= c ≡ C 的 a<=b && b<=c
        in_range = self.range_min <= z <= self.range_max          # 目标是否在可抓取的距离范围内

        # ── 第 6 步：填充 TargetDepth 消息并发布 ──
        # TargetDepth 是你们在 vision_grasp_interfaces 里自定义的消息类型
        # .msg 文件里定义了所有字段，ROS2 自动生成 Python 类，你直接 .xxx 访问
        out = TargetDepth()
        out.header = color_msg.header              # 复制原消息的时间戳和坐标系名（保持时间戳一致性）
        out.class_name = class_name                 # 类别名（字符串，如 "甲骨A"）
        out.class_id = int(class_id)                # 类别 ID（整数，YOLO 内部的类别编号）
        out.confidence = float(conf)                # 置信度（浮点数，0~1）
        out.center_u = int(u)                       # 质心像素列坐标
        out.center_v = int(v)                       # 质心像素行坐标
        out.center_depth = float(z)                 # 质心深度（米）
        # p_cam = (x, y, z) 元组 → 三个字段分别赋值
        out.position_camera.x, out.position_camera.y, out.position_camera.z = p_cam
        # p_grip = (x', y', z') 元组 → 三个字段分别赋值
        out.position_gripper.x, out.position_gripper.y, out.position_gripper.z = p_grip
        out.valid = True                             # 标记此帧有效
        out.in_range = bool(in_range)                # 目标是否在可抓取范围内
        self.target_pub.publish(out)                  # 发布！→ serial_bridge 收到 → 打包串口帧 → STM32

    # ═══════════════════════════════════════════════════════════
    # YOLO 推理（= 把一张图扔进 AI 模型，拿出检测结果）
    # ═══════════════════════════════════════════════════════════

    def _infer_best_target(self, color: np.ndarray):
        """
        输入：一张 BGR 彩色图的 numpy 数组
        输出：(class_id, class_name, confidence, mask) 四元组，或 None（没检测到）
        mask 是 True/False 的二维数组（和原图一样大，True=目标像素）
        """
        # ── model.predict() = 跑 YOLO 推理 ──
        # source=color   → 输入图像（numpy 数组，YOLO 能直接吃）
        # conf=self.conf → 置信度阈值（低于 0.5 的框不要）
        # iou=self.iou   → 非极大抑制阈值（重叠框合并）
        # imgsz=640      → 内部把图 resize 到 640×640 再推理
        # device="cpu"   → 用 CPU 算（带 Nvidia 显卡可以设 "cuda:0"）
        # verbose=False  → 不打印进度条（命令行干净）
        # 返回值是一个 list[Results]，一张图一个元素
        results = self.model.predict(
            source=color, conf=self.conf, iou=self.iou,
            imgsz=self.imgsz, device=self.device, verbose=False)
        if not results:                                      # 空列表（不太可能，但防御一下）
            return None
        res = results[0]                                     # 取第一张图的结果（只传了一张图）

        # res.masks = 分割掩膜数据；res.boxes = 检测框数据
        # res.masks is None = YOLO 不是分割模式（只检测框，没有掩膜）
        # len(res.boxes) == 0 = 没检测到任何目标
        if res.masks is None or res.boxes is None or len(res.boxes) == 0:
            return None

        # ── 提取类别名、图像尺寸 ──
        names = res.names                                    # 字典：{0: "甲骨1", 1: "封泥2", ...}
        h, w = color.shape[:2]                               # 图像高度、宽度（像素）
                                                              # color.shape = (H, W, 3) → [:2] 取前两个 → (H, W)

        best = None                                          # 当前最佳目标（初始空）
        best_conf = -1.0                                     # 当前最高置信度（初始 -1，保证第一个检测到的自动当选）

        # ── YOLO 的 GPU tensor → CPU numpy 数组 ──
        # YOLO 推理结果在 GPU 显存里（tensor 格式），需要搬到 CPU 才能正常访问
        # .cpu()    = GPU → CPU 拷贝
        # .numpy()  = PyTorch tensor → numpy array（C 数组）
        masks = res.masks.data.cpu().numpy()                  # 掩膜数组: (num_detections, H, W)
        cls_arr = res.boxes.cls.cpu().numpy().astype(int)     # 类别 ID 数组: (num_detections,)
                                                               # .astype(int) = 转成整数类型（原始是 float）
        conf_arr = res.boxes.conf.cpu().numpy()                # 置信度数组: (num_detections,)

        # ── 遍历所有检测框，找置信度最高且类别匹配的那个 ──
        for i in range(len(cls_arr)):                        # range(N) = C 的 for(i=0; i<N; i++)
            cid = int(cls_arr[i])                             # 当前框的类别编号
            cname = names.get(cid, str(cid))                  # 查字典：编号 → 类别名；没有就用字符串 "数字"
            # 类别过滤：如果配置了 target_class_names（只要特定类别），且当前类别不在名单里 → 跳过
            if self.target_class_names and cname not in self.target_class_names:
                continue                                      # continue = C 的跳过本次循环，继续下一个

            c = float(conf_arr[i])                            # 当前置信度
            if c <= best_conf:                                # 不是最高 → 跳过
                continue

            m = masks[i]                                      # 当前掩膜（可能和原图尺寸不一样）
            if m.shape != (h, w):                              # 掩膜尺寸 ≠ 原图尺寸 → 需 resize
                import cv2                                     # OpenCV（Python 的 import 可以在函数内部）
                # cv2.resize(源, (宽,高), 插值方法)
                # INTER_NEAREST = 最近邻插值（保留二值掩膜的锐利边缘，不用双线性模糊）
                m = cv2.resize(m, (w, h), interpolation=cv2.INTER_NEAREST)

            # m > 0.5：掩膜值二值化（YOLO 输出的掩膜是 0~1 的浮点概率，>0.5 判为目标像素）
            best = (cid, cname, c, (m > 0.5))                 # 四元组：(类别编号, 类别名, 置信度, 二值掩膜)
            best_conf = c                                      # 更新最高置信度

        return best                                            # 循环结束，返回最佳目标（可能还是 None）


# ═══════════════════════════════════════════════════════════
# 程序入口 = C 的 main() 函数
# ═══════════════════════════════════════════════════════════

def main(args=None) -> None:
    """ROS2 节点入口：初始化 → 创建节点 → spin 死循环 → Ctrl+C 退出 → 清理"""
    rclpy.init(args=args)                 # = HAL_Init + FreeRTOS 初始化
    node = DetectorNode()                  # 创建检测器节点（构造函数里注册了 publisher/subscriber/service）
    try:
        rclpy.spin(node)                   # = osKernelStart()——死循环等待话题消息，自动调所有回调
    except KeyboardInterrupt:              # = Ctrl+C 中断
        pass                               # 什么都不做，直接跳到 finally 清理
    finally:                               # Python 的 finally = C 的 __attribute__((cleanup))——无论如何都会执行
        node.destroy_node()                # 销毁节点（释放 ROS2 资源）
        if rclpy.ok():                     # 如果 ROS2 还没关（有时 Ctrl+C 时已经关了）
            rclpy.shutdown()               # 关闭 ROS2


# Python 约定：直接运行这个文件时 __name__ == "__main__" → 调 main()
# 如果被 import 时 __name__ == "detector_node" → 不调 main()
if __name__ == "__main__":
    main()
