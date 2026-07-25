# -*- coding: utf-8 -*-
"""
serial_bridge_node.py  ——  串口桥节点
唯一持有物理串口的 ROS 2 节点，所有其他节点通过话题与本节点交互。

上行（NUC → STM32）：
    /odometry        nav_msgs/Odometry
                     → CMD_ODOM(0x01)，全程持续按 send_rate_hz 发送
    /target_depth    vision_grasp_interfaces/TargetDepth
                     → CMD_KFS(0x03)        检测类别 + 相机坐标
                     → CMD_KFS_LATERAL_ERR(0x06)  相机坐标（下位机横向误差校正）
                     两帧同时发送，互补。valid=False 时丢弃。

下行（STM32 → NUC）：
    CMD_GO_ZONE_I(0x15)   → /camera_command True       ★ 开摄像头 ★
    CMD_STATUS(0x12) IDLE/DONE → /camera_command False ★ 关摄像头 ★
    CMD_STATUS 其他状态        → /robot_status UInt8   状态机使用
    CMD_ZONE_I_INFO(0x13)      → /zone_i_info  String  JSON 格式
    CMD_DOCK_OK(0x14)          → /dock_ok Bool
    CMD_ACK(0x10)              → 仅打日志
    CMD_DEBUG_HEADING(0x20)    → /debug_heading String  JSON 格式
    CMD_DEBUG_NAV(0x21)        → /debug_nav     String  JSON 格式
"""

import json                         # 序列化/反序列化 JSON（调试数据用 JSON 格式传）
import threading                    # 线程锁，保护串口不被两个线程同时读写

import rclpy                        # Python 版 ROS2 客户端库
from rclpy.node import Node         # ROS2 节点基类（所有节点都继承这个）
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy  # QoS（服务质量），控制话题的可靠性和缓存策略

from nav_msgs.msg import Odometry   # ROS 标准消息：里程计（x,y,z,roll,pitch,yaw）
from std_msgs.msg import Bool, UInt8, String, Float32, Float32MultiArray  # 标准消息类型
                                                                           # Bool=C的uint8_t, UInt8=C的uint8_t, String=C的char*, Float32=C的float, Float32MultiArray=C的float数组

from vision_grasp_interfaces.msg import TargetDepth  # 自定义消息：摄像头检测到的目标位置

# from .protocol import ... = 从同目录下的 protocol.py 引入函数
# "." 表示当前包（relative import）
from .protocol import (
    # 编码函数（python对象 → 串口字节帧）
    encode_odom,                     # 里程计编码：6个float → 24字节
    encode_kfs_single,               # KFS检测编码：类别+坐标 → payload
    encode_kfs_lateral_err,          # KFS横向误差编码：坐标 → payload
    # 解码函数（串口字节帧 → python对象）
    decode_ack,                      # ACK 帧解码
    decode_zone_i_info,              # I区KFS布局解码
    decode_debug_heading,            # 航向保持调试数据解码
    decode_debug_nav,                # 导航调试数据解码
    # 帧解析器类（状态机字节拼帧）
    FrameParser,                     # 逐字节喂入 → 完整的帧吐出来
    # 命令码枚举（和STM32的rc_cmd_t一致）
    CMD_ACK,                         # 0x10
    CMD_STATUS,                      # 0x12
    CMD_ZONE_I_INFO,                 # 0x13
    CMD_DOCK_OK,                     # 0x14
    CMD_GO_ZONE_I,                   # 0x15
    CMD_DEBUG_HEADING,               # 0x20
    CMD_DEBUG_NAV,                   # 0x21
    # STM32 状态枚举
    RC_STATE_IDLE,                   # 0 = 空闲
    RC_STATE_DONE,                   # 4 = 任务完成
    # 坐标转换工具
    quat_to_euler_deg,               # 四元数 → 欧拉角（度）
)

# 尝试导入 pyserial（串口库）。如果没装 → serial = None，节点启动时会报日志提醒
try:
    import serial                    # pip install pyserial
except ImportError:
    serial = None


class SerialBridge(Node):            # 继承 Node = 定义一个 ROS2 节点 (= FreeRTOS 的 Can_Task)
    def __init__(self) -> None:      # __init__ = 构造函数（节点启动时自动调一次）
        super().__init__("serial_bridge")  # 注册节点名（终端 ros2 node list 能看到这个名字）

        # ── 参数声明 ──────────────────────────────────────────────────────
        # declare_parameter() 声明这个节点接受哪些可配参数（运行时可通过 YAML 文件或命令行覆盖）
        # declare_parameter("参数名", 默认值) — 类比 C 里带默认值的 #define
        self.declare_parameter("port",                "/dev/ttyUSB_online")  # 串口设备路径（Linux下是/dev/ttyXXX）
        self.declare_parameter("baudrate",            115200)                # 波特率
        self.declare_parameter("send_rate_hz",        50.0)    # ODOM 上行发送频率（Hz）
        self.declare_parameter("read_poll_hz",        200.0)   # 串口读取轮询频率（Hz，比发送快是为了收数据不丢帧）
        self.declare_parameter("odom_topic",          "/odometry")           # FAST-LIO2 输出的里程计话题
        self.declare_parameter("target_topic",        "/target_depth")       # 摄像头检测结果话题
        self.declare_parameter("camera_cmd_topic",    "/camera_command")     # 下发摄像头开关指令
        self.declare_parameter("robot_status_topic",  "/robot_status")       # STM32状态上报
        self.declare_parameter("dock_ok_topic",       "/dock_ok")            # R1对接成功
        self.declare_parameter("zone_i_info_topic",   "/zone_i_info")        # I区KFS布局
        self.declare_parameter("debug_heading_topic", "/debug_heading")      # 航向PID调试
        self.declare_parameter("debug_nav_topic",     "/debug_nav")          # 导航PID调试
        # ── PC 控制指令话题（新增：键盘遥控 → 串口） ──
        self.declare_parameter("chassis_cmd_topic",   "/chassis_cmd")       # 底盘速度 [Vx,Vy,Vw]
        self.declare_parameter("kfs_cmd_topic",       "/kfs_cmd")           # KFS取放指令
        self.declare_parameter("lift_cmd_topic",      "/lift_cmd")          # 抬升速度
        self.declare_parameter("flow_cmd_topic",      "/flow_cmd")          # 流程函数触发
        self.declare_parameter("zone_cmd_topic",      "/zone_cmd")          # 业务zone触发
        self.declare_parameter("pc_estop_topic",      "/pc_estop")          # PC急停
        self.declare_parameter("weapon_cmd_topic",    "/weapon_cmd")         # 武器toggle
        self.declare_parameter("kfs_pos_cmd_topic",   "/kfs_pos_cmd")       # KFS档位调节

        # 读取参数值（get_parameter().value = 取最终生效的配置值）
        self.port      = self.get_parameter("port").value                    # 串口设备路径
        self.baud      = int(self.get_parameter("baudrate").value)          # 波特率（转整数）
        self.send_rate = float(self.get_parameter("send_rate_hz").value)    # 发送频率（转浮点）
        poll_hz        = float(self.get_parameter("read_poll_hz").value)    # 轮询频率

        # ── 串口初始化 ──────────────────────────────────────────────────────
        self._ser      = None                    # pyserial 串口对象，初始 None（还没打开）
        self._ser_lock = threading.Lock()        # 线程锁（互斥锁）——防止写线程和读线程同时操作串口
                                                 # threading.Lock() 相当于 FreeRTOS 的 mutex
        self._open_serial()                     # 尝试打开串口

        # 下行帧解析器——和下位机 upper_pc_protocol.c 的 rc_feed_byte 状态机是一对
        # FrameParser 内部有字节缓冲区，每收到一个字节就尝试拼成一帧
        self._parser = FrameParser()

        # ── 创建发布者（= 声明"我会往这些话题发消息"） ──
        # create_publisher(消息类型, 话题名, 队列长度)
        # 队列长度：如果下游消费速度跟不上，最多缓存 10 帧（超过就丢）
        self.cam_cmd_pub    = self.create_publisher(Bool,   self.get_parameter("camera_cmd_topic").value,    10)
        self.status_pub     = self.create_publisher(UInt8,  self.get_parameter("robot_status_topic").value,  10)
        self.dock_ok_pub    = self.create_publisher(Bool,   self.get_parameter("dock_ok_topic").value,       10)
        self.zone_info_pub  = self.create_publisher(String, self.get_parameter("zone_i_info_topic").value,   10)
        self.dbg_heading_pub = self.create_publisher(String, self.get_parameter("debug_heading_topic").value, 10)
        self.dbg_nav_pub    = self.create_publisher(String, self.get_parameter("debug_nav_topic").value,     10)

        # ── 创建订阅者（= 声明"我会从这些话题收消息"） ──
        # QoS（Quality of Service）配置：里程计数据量大且实时性强 → 用 BEST_EFFORT（丢了不重传）
        sensor_qos = QoSProfile(
            depth=10,                                    # 缓存深度
            reliability=ReliabilityPolicy.BEST_EFFORT,   # 尽最大努力投递，丢了不重发（省带宽）
            history=HistoryPolicy.KEEP_LAST)              # 只保留最新的一帧

        # self._latest_odom 存储最新收到的里程计消息
        # Odometry | None 是 Python 的类型注解（标注这个变量可以是 Odometry 或 None）
        self._latest_odom: Odometry | None = None

        # 订阅里程计（接收 FAST-LIO2 输出的定位结果）
        self.create_subscription(Odometry, self.get_parameter("odom_topic").value,
                                  self._on_odom, sensor_qos)  # 收到数据 → 调 _on_odom 回调

        # 订阅摄像头检测结果（接收 YOLO 输出的目标位置）
        self.create_subscription(TargetDepth, self.get_parameter("target_topic").value,
                                  self._on_target, 10)

        # ── PC 控制指令订阅（键盘遥控 → 话题 → 这里打包成串口帧发 STM32） ──
        # 底盘：Float32MultiArray [Vx,Vy,Vw]
        self.create_subscription(Float32MultiArray, self.get_parameter("chassis_cmd_topic").value,
                                  self._on_chassis_cmd, 10)
        # KFS取放：Float32 动作号
        self.create_subscription(Float32, self.get_parameter("kfs_cmd_topic").value,
                                  self._on_kfs_cmd, 10)
        # 抬升：Float32 速度
        self.create_subscription(Float32, self.get_parameter("lift_cmd_topic").value,
                                  self._on_lift_cmd, 10)
        # 流程函数：Float32 流程号
        self.create_subscription(Float32, self.get_parameter("flow_cmd_topic").value,
                                  self._on_flow_cmd, 10)
        # 业务zone：Float32 zone号
        self.create_subscription(Float32, self.get_parameter("zone_cmd_topic").value,
                                  self._on_zone_cmd, 10)
        # PC急停：Float32 1/0
        self.create_subscription(Float32, self.get_parameter("pc_estop_topic").value,
                                  self._on_pc_estop, 10)
        # 武器toggle：Float32 设备号
        self.create_subscription(Float32, self.get_parameter("weapon_cmd_topic").value,
                                  self._on_weapon_cmd, 10)
        # KFS档位：Float32MultiArray [设备号, 方向]
        self.create_subscription(Float32MultiArray, self.get_parameter("kfs_pos_cmd_topic").value,
                                  self._on_kfs_pos_cmd, 10)

        # ── 创建定时器（= FreeRTOS 的 osDelay + 循环体） ──
        # max(send_rate, 1.0) 防止除零（如果配成 0 就变成 1Hz）
        # 1.0 / 50.0 = 0.02 秒 = 50Hz
        self.create_timer(1.0 / max(self.send_rate, 1.0), self._tick_odom)  # 50Hz：定时发里程计到 STM32
        self.create_timer(1.0 / max(poll_hz, 1.0),        self._tick_read)  # 200Hz：从串口读 STM32 回传的数据

        # f-string 嵌变量打启动日志（Python 的 f"xxx{变量}" 和 C 的 printf("%d",x) 一样效果）
        self.get_logger().info(
            f"serial_bridge 启动  port={self.port}  baud={self.baud}  "
            f"odom_rate={self.send_rate} Hz")

    # ═══════════════════════════════════════════════════════════ 串口管理

    def _open_serial(self) -> None:
        """打开串口。pyserial.Serial() = C 的 HAL_UART_Init()。
        失败不崩溃，只打日志并保持 _ser = None，下次 _write 时会重试。"""
        if serial is None:                 # pip 没装 pyserial
            self.get_logger().error("未安装 pyserial（pip install pyserial）")
            return
        try:
            self._ser = serial.Serial(self.port, self.baud, timeout=0)  # timeout=0 = 非阻塞读
            self.get_logger().info(f"串口已打开：{self.port} @ {self.baud}")
        except Exception as exc:          # 捕获所有异常（= C 里 if (HAL_OK != ret)）
            self._ser = None
            self.get_logger().error(f"串口打开失败：{exc}")

    def _write(self, frame: bytes) -> None:
        """向串口写入一帧（= C 里 HAL_UART_Transmit）
        bytes = Python 的字节数组，类比 C 的 uint8_t buffer[]
        失败时标记串口无效（_ser = None），下次 _open_serial 重连"""
        if self._ser is None:             # 串口还没打开
            self._open_serial()           # 尝试打开
            if self._ser is None: return  # 还是打不开 → 放弃
        try:
            with self._ser_lock:          # with = 获取锁...退出 with 时自动释放锁
                self._ser.write(frame)    # 写字节数组到串口
        except Exception as exc:
            self.get_logger().warn(f"串口写入失败：{exc}")
            self._ser = None              # 标记无效，下次重连

    # ═══════════════════════════════════════════════════════════ 上行（NUC → STM32）

    def _on_odom(self, msg: Odometry) -> None:
        """里程计回调：只缓存最新值，不立即发送。
        由定时器 _tick_odom 限频发送（50Hz），避免 FAST-LIO2 发多快我就发多快（带宽容不下）。"""
        self._latest_odom = msg           # 缓存最新一帧里程计

    def _tick_odom(self) -> None:
        """定时器 50Hz：把缓存的里程计打包成 CMD_ODOM(0x01) 帧发 STM32。
        坐标系映射和偏移补偿全在 STM32 侧处理——上位机直接传原始值。"""
        msg = self._latest_odom
        if msg is None: return            # 还没收到过里程计 → 跳过
        pos  = msg.pose.pose.position     # pos = (x, y, z) 米
        q    = msg.pose.pose.orientation  # q = (x,y,z,w) 四元数
        # 四元数 → 欧拉角（度）。这个函数在同目录 protocol.py 里
        roll, pitch, yaw = quat_to_euler_deg(q.x, q.y, q.z, q.w)
        # encode_odom() 在 protocol.py 里，把 6 个 float 打包成 24 字节 payload
        frame = encode_odom(pos.x, pos.y, pos.z, roll, pitch, yaw)
        self._write(frame)                # 写串口 → STM32 收到 CMD_ODOM → 更新 g_sensor_data.odom

    def _on_target(self, msg: TargetDepth) -> None:
        """收到摄像头检测结果 → 同时发送两个帧给 STM32：
        ① CMD_KFS(0x03)：类别ID + 相机坐标 → 供下位机抓取规划
        ② CMD_KFS_LATERAL_ERR(0x06)：纯相机坐标 → 供下位机横向误差校正
        两帧互补，各自对应 STM32 的不同处理流程。
        valid=False（深度无效帧）→ 不发送。"""
        if not msg.valid: return          # 检测结果无效（深度为0、置信度不够等）
        c = msg.position_camera           # 相机坐标系下的 3D 坐标

        self._write(encode_kfs_single(msg.class_id, c.x, c.y, c.z))  # ① 类别+坐标
        self._write(encode_kfs_lateral_err(c.x, c.y, c.z))           # ② 纯坐标

    # ═══════════════════════════════════════════════════════════ PC 控制指令编码
    # 这些函数负责：收到 ROS2 话题消息 → 打包成串口协议帧 → 发给 STM32
    # 帧格式：[0xA5][0x5A][CMD][LEN_LO][LEN_HI][PAYLOAD][CHKSUM?]
    # 和 STM32 的 upper_pc_protocol.c 帧格式完全对齐

    def _on_chassis_cmd(self, msg: Float32MultiArray) -> None:
        """PC→STM32 0x30：底盘速度指令 Vx,Vy,Vw（3个float = 12字节）"""
        if len(msg.data) < 3: return      # 数据不完整 → 忽略
        import struct  # struct.pack('<fff',...) = 把小端序的3个float打包成12字节
        payload = struct.pack('<fff', msg.data[0], msg.data[1], msg.data[2])
        # bytes([0xA5, 0x5A, 0x30, 0x0C, 0x00]) = 帧头+CMD+长度(12)
        frame = bytes([0xA5, 0x5A, 0x30, 0x0C, 0x00]) + payload + bytes([0x00])
        self._write(frame)

    def _on_kfs_cmd(self, msg: Float32) -> None:
        """PC→STM32 0x32：KFS 取放动作（1字节：0=停 1=取 2=放）
        int(msg.data) & 0xFF = 取 float 的整数部分并截断到 0~255"""
        frame = bytes([0xA5, 0x5A, 0x32, 0x01, 0x00, int(msg.data) & 0xFF, 0x00])
        self._write(frame)

    def _on_lift_cmd(self, msg: Float32) -> None:
        """PC→STM32 0x33：抬升速度（1个float = 4字节）"""
        import struct
        val = struct.pack('<f', msg.data)  # float → 4字节小端序
        frame = bytes([0xA5, 0x5A, 0x33, 0x04, 0x00]) + val + bytes([0x00])
        self._write(frame)

    def _on_flow_cmd(self, msg: Float32) -> None:
        """PC→STM32 0x34：流程触发（1字节：1=取KFS 2=放KFS 3=上台阶 4=下台阶 5=上R1 6=上坡 7=摄像头调试）"""
        frame = bytes([0xA5, 0x5A, 0x34, 0x01, 0x00, int(msg.data) & 0xFF, 0x00])
        self._write(frame)

    def _on_zone_cmd(self, msg: Float32) -> None:
        """PC→STM32 0x35：业务zone启动（1字节：1=一区 2=二区 3=三区 4=三区预备）"""
        frame = bytes([0xA5, 0x5A, 0x35, 0x01, 0x00, int(msg.data) & 0xFF, 0x00])
        self._write(frame)

    def _on_pc_estop(self, msg: Float32) -> None:
        """PC→STM32 0x31：急停（0字节payload）"""
        frame = bytes([0xA5, 0x5A, 0x31, 0x00, 0x00, 0x00])
        self._write(frame)

    def _on_weapon_cmd(self, msg: Float32) -> None:
        """PC→STM32 0x36：武器 toggle（1字节：设备号 1=吸盘1 2=吸盘2 3=吸盘3 4=吸盘4 6=夹爪 7=舵机）"""
        dev = int(msg.data) & 0xFF         # float→int→截断8位
        frame = bytes([0xA5, 0x5A, 0x36, 0x01, 0x00, dev, 0x00])
        self._write(frame)

    def _on_kfs_pos_cmd(self, msg: Float32MultiArray) -> None:
        """PC→STM32 0x37~0x3B：KFS档位调节（[设备号, 方向]）
        设备号 1=three_kfs 2=kfs_spin 3=main_lift 4=flex 5=flex_mode
        方向 0=减 1=加
        CMD = 0x36 + 设备号 → 1→0x37, 2→0x38, ..., 5→0x3B"""
        if len(msg.data) < 2: return
        dev = int(msg.data[0]) & 0xFF
        dir_val = int(msg.data[1]) & 0xFF
        cmd = 0x36 + dev                   # 0x36 + 1 = 0x37（three_kfs）
        frame = bytes([0xA5, 0x5A, cmd, 0x01, 0x00, dir_val, 0x00])
        self._write(frame)

    # ═══════════════════════════════════════════════════════════ 下行（STM32 → NUC）

    def _tick_read(self) -> None:
        """定时器 200Hz：从串口读数据 → 喂给 FrameParser → 收到完整帧 → _dispatch 分发"""
        if self._ser is None: return       # 串口没开
        try:
            with self._ser_lock:           # 拿锁（和 _write 互斥）
                n    = self._ser.in_waiting  # 串口缓冲区里有多少字节等着读（= C 里 UART 的 RX buffer 长度）
                data = self._ser.read(n) if n else b""  # 读 n 个字节；如果没有数据就返回空 bytes（b"" = 空字节串）
        except Exception as exc:
            self.get_logger().warn(f"串口读取失败：{exc}")
            self._ser = None               # 标记无效，重连
            return
        # self._parser.feed(data)：逐字节喂给 FrameParser（类似 STM32 的 rc_feed_byte）
        # feed() 返回一个可迭代对象——每拼出一帧就 yield 那一帧
        for frame in self._parser.feed(data):
            self._dispatch(frame)          # 分派到对应 ROS2 话题

    def _dispatch(self, frame) -> None:
        """
        分发下行帧到对应 ROS2 话题
        和 STM32 的 upper_pc_protocol.c 里的 dispatch_frame 完全对称
        frame.cmd = 命令码
        frame.payload = 载荷字节
        """
        cmd = frame.cmd

        if cmd == CMD_GO_ZONE_I:           # 0x15：STM32 请求进入 I 区
            self.cam_cmd_pub.publish(Bool(data=True))     # 通知摄像头节点：开始检测
            self.get_logger().info("收到 CMD_GO_ZONE_I(0x15) → 开摄像头")

        elif cmd == CMD_STATUS:            # 0x12：STM32 上报当前状态
            if len(frame.payload) < 1: return
            state = frame.payload[0]        # 状态字节
            self.status_pub.publish(UInt8(data=int(state)))  # 发布到 /robot_status
            # IDLE(0) 或 DONE(4) → 任务结束 → 通知摄像头：停止检测
            if state in (RC_STATE_IDLE, RC_STATE_DONE):
                self.cam_cmd_pub.publish(Bool(data=False))
                self.get_logger().info(
                    f"CMD_STATUS state={state}（{'IDLE' if state == RC_STATE_IDLE else 'DONE'}）→ 关摄像头")

        elif cmd == CMD_DOCK_OK:           # 0x14：R1 对接成功
            self.dock_ok_pub.publish(Bool(data=True))
            self.get_logger().info("收到 CMD_DOCK_OK(0x14)：R1 对接成功")

        elif cmd == CMD_ZONE_I_INFO:       # 0x13：I 区 KFS 布局
            info = decode_zone_i_info(frame.payload)
            if info is not None:
                # Python list comprehension（列表推导式）：一行生成一个列表
                # [{"block_id": d.block_id, ...} for d in info.detections] = 遍历 info.detections，每个元素生成一个字典
                payload_json = json.dumps({
                    "detections": [
                        {"block_id": d.block_id, "kfs_type": d.kfs_type}
                        for d in info.detections
                    ]
                })
                self.zone_info_pub.publish(String(data=payload_json))

        elif cmd == CMD_ACK:               # 0x10：确认帧
            ack = decode_ack(frame.payload)
            if ack is not None:
                self.get_logger().info(
                    f"收到 CMD_ACK(0x10)：acked_cmd=0x{ack.acked_cmd:02X} "
                    f"code={'OK' if ack.code == 0 else 'ERR'}")

        elif cmd == CMD_DEBUG_HEADING:     # 0x20：航向保持 PID 调试数据
            dbg = decode_debug_heading(frame.payload)
            if dbg is not None:
                # round(value, 3) = Python 的四舍五入，保留 3 位小数
                msg = String(data=json.dumps({
                    "yaw_ref_deg":   round(dbg.yaw_ref_deg, 3),
                    "yaw_deg":       round(dbg.yaw_deg, 3),
                    "err_deg":       round(dbg.err_deg, 3),
                    "i_term":        round(dbg.i_term, 4),
                    "output":        round(dbg.output, 4),
                    "yaw_rate_dps":  round(dbg.yaw_rate_dps, 3),
                }))
                self.dbg_heading_pub.publish(msg)

        elif cmd == CMD_DEBUG_NAV:         # 0x21：导航到点调试数据
            dbg = decode_debug_nav(frame.payload)
            if dbg is not None:
                msg = String(data=json.dumps({
                    "ex":      round(dbg.ex, 4),
                    "ey":      round(dbg.ey, 4),
                    "dist":    round(dbg.dist, 4),
                    "zone":    dbg.zone,
                    "vy_fwd":  round(dbg.vy_fwd, 4),
                    "vw_str":  round(dbg.vw_str, 4),
                }))
                self.dbg_nav_pub.publish(msg)

        else:
            self.get_logger().debug(f"忽略未处理下行 CMD 0x{cmd:02X}")


def main(args=None) -> None:
    """程序入口（= C 的 main() 函数）。在 setup.py 里注册为 serial_bridge_node 可执行程序。"""
    rclpy.init(args=args)                # 初始化 ROS2
    node = SerialBridge()                # 创建节点（构造函数里注册了所有 publisher/subscriber/timer）
    try:
        rclpy.spin(node)                 # 死循环（= osKernelStart）——让 ROS2 框架接管，自动调所有回调
    except KeyboardInterrupt:            # Ctrl+C
        pass                             # 什么都不做，跳到 finally
    finally:
        node.destroy_node()              # 销毁节点
        if rclpy.ok():
            rclpy.shutdown()             # 关闭 ROS2


if __name__ == "__main__":
    main()
