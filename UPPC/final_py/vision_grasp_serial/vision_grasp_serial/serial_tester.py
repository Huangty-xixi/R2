#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
serial_tester.py  ——  serial_bridge_node 自动化测试脚本
========================================================
通过虚拟串口对（socat）模拟下位机，自动验证串口桥节点
的上行打帧逻辑和下行解析逻辑，输出逐项 PASS / FAIL。

━━━━ 使用前三步准备 ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

  步骤 1 ── 安装 socat（若未安装）：
    sudo apt install socat

  步骤 2 ── 终端 A：创建虚拟串口对（保持运行）：
    socat -d -d \\
      pty,raw,echo=0,link=/tmp/vserial_bridge \\
      pty,raw,echo=0,link=/tmp/vserial_tester
    # 创建成功后屏幕会打印两个 /dev/pts/N，
    # 软链接 /tmp/vserial_bridge 和 /tmp/vserial_tester 会自动建立。

  步骤 3 ── 终端 B：启动 serial_bridge_node（连接到 bridge 端）：
    source ~/RC2026/install/setup.bash
    ros2 run vision_grasp_serial serial_bridge \\
      --ros-args -p port:=/tmp/vserial_bridge -p baudrate:=115200

  步骤 4 ── 终端 C：运行本脚本（连接到 tester 端）：
    source ~/RC2026/install/setup.bash
    python3 serial_tester.py
    # 或指定自定义端口：
    # python3 serial_tester.py --port /tmp/vserial_tester

━━━━ 测试覆盖 ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

  Phase 1  下行帧解析（模拟下位机 → 验证 ROS 话题）
      T1.1   CMD_GO_ZONE_I   → /camera_command = True
      T1.2   CMD_STATUS IDLE → /camera_command = False
      T1.3   CMD_STATUS DONE → /camera_command = False
      T1.4   CMD_STATUS MOVING   → /robot_status = 1
      T1.5   CMD_STATUS GRABBING → /robot_status = 3
      T1.6   CMD_DOCK_OK     → /dock_ok = True
      T1.7   CMD_ACK         → 不触发 ROS 话题（仅日志）
      T1.8   校验和损坏的帧  → 丢弃，后续好帧恢复正常
      T1.9   前置噪声字节    → 自动重同步
      T1.10  CMD_ZONE_I_INFO → /zone_i_info JSON 格式正确

  Phase 2  上行帧格式（发布 ROS 消息 → 验证串口字节）
      T2.1   /odometry → CMD_ODOM 帧同步头正确
      T2.2   CMD_ODOM 载荷 24 字节，p0=pos.x，p1=pos.y
      T2.3   CMD_ODOM yaw 为角度制（四元数已转换）
      T2.4   CMD_ODOM 帧校验和正确
      T2.5   /odometry 上行帧率约 50 Hz（定时器驱动）
      T2.6   /target_depth(valid=True) → CMD_KFS + CMD_KFS_LATERAL_ERR
      T2.7   CMD_KFS 载荷 class_id / xyz 字段正确
      T2.8   /target_depth(valid=False) → 不发任何帧
"""

import argparse
import json
import math
import queue
import struct
import sys
import threading
import time

# ─── 颜色输出 ────────────────────────────────────────────────────────────────
_G = "\033[92m"   # 绿
_R = "\033[91m"   # 红
_Y = "\033[93m"   # 黄
_B = "\033[94m"   # 蓝
_N = "\033[0m"    # 重置

PASS = f"{_G}PASS{_N}"
FAIL = f"{_R}FAIL{_N}"
SKIP = f"{_Y}SKIP{_N}"
INFO = f"{_B}INFO{_N}"

# ─── 协议常量（内联，无需 import protocol.py）────────────────────────────────
HEAD0, HEAD1 = 0xA5, 0x5A

CMD_ODOM              = 0x01
CMD_KFS               = 0x03
CMD_KFS_LATERAL_ERR   = 0x06
CMD_ACK               = 0x10
CMD_STATUS            = 0x12
CMD_ZONE_I_INFO       = 0x13
CMD_DOCK_OK           = 0x14
CMD_GO_ZONE_I         = 0x15
CMD_DEBUG_HEADING     = 0x20
CMD_DEBUG_NAV         = 0x21

RC_STATE_IDLE     = 0
RC_STATE_MOVING   = 1
RC_STATE_GRABBING = 3
RC_STATE_DONE     = 4


def calc_checksum(cmd: int, payload: bytes) -> int:
    n = len(payload)
    chk = cmd ^ (n & 0xFF) ^ ((n >> 8) & 0xFF)
    for b in payload:
        chk ^= b
    return chk & 0xFF


def build_frame(cmd: int, payload: bytes = b"") -> bytes:
    n = len(payload)
    chk = calc_checksum(cmd, payload)
    return bytes([HEAD0, HEAD1, cmd, n & 0xFF, (n >> 8) & 0xFF]) + payload + bytes([chk])


class FrameParser:
    """增量式帧解析器（与 protocol.py 逻辑相同）。"""
    def __init__(self):
        self._buf = bytearray()

    def reset(self):
        self._buf.clear()

    def feed(self, data: bytes):
        self._buf.extend(data)
        frames = []
        while True:
            frame, consumed = self._try_parse()
            if consumed == 0:
                break
            del self._buf[:consumed]
            if frame is not None:
                frames.append(frame)
        return frames

    def _try_parse(self):
        buf = self._buf
        if len(buf) < 2:
            return None, 0
        if not (buf[0] == HEAD0 and buf[1] == HEAD1):
            return None, 1
        if len(buf) < 5:
            return None, 0
        cmd    = buf[2]
        length = buf[3] | (buf[4] << 8)
        if length > 64:
            return None, 1
        total = 5 + length + 1
        if len(buf) < total:
            return None, 0
        payload  = bytes(buf[5:5 + length])
        recv_chk = buf[total - 1]
        if recv_chk != calc_checksum(cmd, payload):
            return None, 1
        return (cmd, payload), total


# ─── 测试结果记录 ─────────────────────────────────────────────────────────────
_results: list[tuple[str, bool]] = []


def check(name: str, cond: bool, detail: str = "") -> bool:
    _results.append((name, cond))
    status  = PASS if cond else FAIL
    detail_s = f"  → {detail}" if detail else ""
    print(f"  [{status}] {name}{detail_s}")
    return cond


def skip(name: str, reason: str = "") -> None:
    _results.append((name, None))
    r = f"  ({reason})" if reason else ""
    print(f"  [{SKIP}] {name}{r}")


# ─── ROS 辅助节点 ─────────────────────────────────────────────────────────────
try:
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Bool, UInt8, String
    HAS_ROS = True
except ImportError:
    HAS_ROS = False

try:
    from vision_grasp_interfaces.msg import TargetDepth
    HAS_TARGET_DEPTH = True
except ImportError:
    HAS_TARGET_DEPTH = False

try:
    import serial as pyserial
    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False


class TesterNode(Node):
    """ROS 节点：订阅 serial_bridge 发布的话题，发布触发上行的话题。"""

    def __init__(self):
        super().__init__("serial_tester")
        self._qs = {
            "cam_cmd":   queue.Queue(),
            "status":    queue.Queue(),
            "dock_ok":   queue.Queue(),
            "zone_info": queue.Queue(),
        }
        self.create_subscription(
            Bool,   "/camera_command", lambda m: self._qs["cam_cmd"].put(m.data),   10)
        self.create_subscription(
            UInt8,  "/robot_status",   lambda m: self._qs["status"].put(m.data),    10)
        self.create_subscription(
            Bool,   "/dock_ok",        lambda m: self._qs["dock_ok"].put(m.data),   10)
        self.create_subscription(
            String, "/zone_i_info",    lambda m: self._qs["zone_info"].put(m.data), 10)

        self.odom_pub = self.create_publisher(Odometry, "/odometry", 10)
        if HAS_TARGET_DEPTH:
            self.target_pub = self.create_publisher(TargetDepth, "/target_depth", 10)

    def wait(self, key: str, timeout: float = 1.2):
        """阻塞等待指定话题收到消息，超时返回 None。"""
        try:
            return self._qs[key].get(timeout=timeout)
        except queue.Empty:
            return None

    def clear(self):
        """清空所有话题缓存，用于隔离每个测试用例。"""
        for q in self._qs.values():
            while not q.empty():
                try:
                    q.get_nowait()
                except queue.Empty:
                    break

    # ── 上行消息构造 ──────────────────────────────────────────────────────────
    def pub_odom(self, x=1.5, y=-0.8, yaw_rad=1.234,
                 vx=0.3, vy=0.0, omega=0.1) -> Odometry:
        msg = Odometry()
        msg.header.frame_id = "odom"
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        qz = math.sin(yaw_rad / 2.0)
        qw = math.cos(yaw_rad / 2.0)
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.twist.twist.linear.x  = vx
        msg.twist.twist.linear.y  = vy
        msg.twist.twist.angular.z = omega
        self.odom_pub.publish(msg)
        return msg

    def pub_target(self, class_id=3, cx=0.02, cy=-0.05, cz=0.65,
                   center_depth=0.65, in_range=True, valid=True):
        if not HAS_TARGET_DEPTH:
            return None
        msg = TargetDepth()
        msg.header.frame_id = "camera_color_optical_frame"
        msg.class_id        = class_id
        msg.class_name      = "de_seal"
        msg.confidence      = 0.95
        msg.center_u        = 600
        msg.center_v        = 400
        msg.center_depth    = center_depth
        msg.position_camera.x = cx
        msg.position_camera.y = cy
        msg.position_camera.z = cz
        msg.position_gripper.x = cx
        msg.position_gripper.y = cy
        msg.position_gripper.z = cz
        msg.valid    = valid
        msg.in_range = in_range
        self.target_pub.publish(msg)
        return msg


# ─── 主测试逻辑 ───────────────────────────────────────────────────────────────
def run_tests(ser, node: TesterNode, parser: FrameParser) -> None:

    def send(frame: bytes) -> None:
        ser.write(frame)
        ser.flush()

    def drain(reset_parser: bool = True) -> None:
        """丢弃未读串口数据，可选择同时重置帧解析器。"""
        time.sleep(0.12)
        ser.reset_input_buffer()
        if reset_parser:
            parser.reset()

    def read_frames(timeout: float = 0.8) -> list:
        """从串口读取一段时间，返回解析出的帧列表。"""
        deadline = time.monotonic() + timeout
        frames   = []
        while time.monotonic() < deadline:
            raw = ser.read(512)
            if raw:
                frames.extend(parser.feed(raw))
        return frames

    # ══════════════════════════════════════════════════════════════════════════
    print(f"\n{_B}─── Phase 1  下行帧解析（模拟下位机 → ROS 话题）{_N}")

    # T1.1  CMD_GO_ZONE_I → /camera_command = True
    node.clear()
    send(build_frame(CMD_GO_ZONE_I, b""))
    val = node.wait("cam_cmd")
    check("T1.1  CMD_GO_ZONE_I(0x15) → /camera_command = True",
          val is True, f"got {val}")

    # T1.2  CMD_STATUS IDLE → /camera_command = False
    node.clear()
    send(build_frame(CMD_STATUS, bytes([RC_STATE_IDLE])))
    val = node.wait("cam_cmd")
    check("T1.2  CMD_STATUS(IDLE=0) → /camera_command = False",
          val is False, f"got {val}")

    # T1.3  CMD_STATUS DONE → /camera_command = False
    node.clear()
    send(build_frame(CMD_STATUS, bytes([RC_STATE_DONE])))
    val = node.wait("cam_cmd")
    check("T1.3  CMD_STATUS(DONE=4) → /camera_command = False",
          val is False, f"got {val}")

    # T1.4  CMD_STATUS MOVING → /robot_status = 1
    node.clear()
    send(build_frame(CMD_STATUS, bytes([RC_STATE_MOVING])))
    val = node.wait("status")
    check("T1.4  CMD_STATUS(MOVING=1) → /robot_status = 1",
          val == RC_STATE_MOVING, f"got {val}")

    # T1.5  CMD_STATUS GRABBING → /robot_status = 3
    node.clear()
    send(build_frame(CMD_STATUS, bytes([RC_STATE_GRABBING])))
    val = node.wait("status")
    check("T1.5  CMD_STATUS(GRABBING=3) → /robot_status = 3",
          val == RC_STATE_GRABBING, f"got {val}")

    # T1.6  CMD_DOCK_OK → /dock_ok = True
    node.clear()
    send(build_frame(CMD_DOCK_OK, b""))
    val = node.wait("dock_ok")
    check("T1.6  CMD_DOCK_OK(0x14) → /dock_ok = True",
          val is True, f"got {val}")

    # T1.7  CMD_ACK → 不触发任何 ROS 话题
    node.clear()
    send(build_frame(CMD_ACK, bytes([CMD_ODOM, 0])))
    cam_val    = node.wait("cam_cmd", timeout=0.4)
    status_val = node.wait("status",  timeout=0.2)
    check("T1.7  CMD_ACK(0x10) → 不触发 ROS 话题（仅日志）",
          cam_val is None and status_val is None)

    # T1.8  校验损坏帧 → 丢弃，后续好帧正常
    node.clear()
    bad = bytearray(build_frame(CMD_DOCK_OK, b""))
    bad[-1] ^= 0xFF           # 破坏校验和
    send(bytes(bad))
    no_val = node.wait("dock_ok", timeout=0.5)
    send(build_frame(CMD_DOCK_OK, b""))    # 坏帧紧跟好帧
    ok_val = node.wait("dock_ok", timeout=1.2)
    check("T1.8  校验损坏帧丢弃，后续好帧正常恢复",
          no_val is None and ok_val is True,
          f"bad_result={no_val}, good_result={ok_val}")

    # T1.9  前置噪声 → 自动重同步
    node.clear()
    noise = bytes([0x00, 0xFF, 0xA5, 0x12, 0x99, 0x5A])  # 非帧头垃圾
    send(noise + build_frame(CMD_GO_ZONE_I, b""))
    val = node.wait("cam_cmd", timeout=1.5)
    check("T1.9  前置噪声字节后自动重同步",
          val is True, f"got {val}")

    # T1.10  CMD_ZONE_I_INFO → /zone_i_info JSON 格式正确
    node.clear()
    zone_pl = bytes([2, 3, 2, 7, 1])   # num=2, (block3 kfs_type2), (block7 kfs_type1)
    send(build_frame(CMD_ZONE_I_INFO, zone_pl))
    val = node.wait("zone_info", timeout=1.2)
    try:
        obj = json.loads(val) if val else None
        dets = obj.get("detections", []) if obj else []
        ok = (len(dets) == 2 and
              dets[0]["block_id"] == 3 and dets[0]["kfs_type"] == 2 and
              dets[1]["block_id"] == 7 and dets[1]["kfs_type"] == 1)
    except Exception:
        ok = False
    check("T1.10 CMD_ZONE_I_INFO(0x13) → /zone_i_info JSON 内容正确",
          ok, f"got {str(val)[:80] if val else None}")

    # ══════════════════════════════════════════════════════════════════════════
    print(f"\n{_B}─── Phase 2  上行帧格式（ROS 话题 → 串口字节）{_N}")

    TEST_X, TEST_Y, TEST_YAW_RAD = 1.5, -0.8, 1.234
    TEST_YAW_DEG = math.degrees(TEST_YAW_RAD)

    # T2.1  同步头与 CMD 字节
    drain()
    node.pub_odom(x=TEST_X, y=TEST_Y, yaw_rad=TEST_YAW_RAD)
    time.sleep(0.08)   # 定时器 50Hz → 最长等 20ms
    frames = read_frames(timeout=0.4)
    odom_f = [(cmd, pl) for cmd, pl in frames if cmd == CMD_ODOM]
    check("T2.1  /odometry → 串口收到 CMD_ODOM(0x01) 帧",
          len(odom_f) > 0, f"共收到 {len(odom_f)} 帧")

    if odom_f:
        _, pl = odom_f[-1]
        p0, p1, z, roll, pitch, yaw = struct.unpack("<6f", pl)

        # T2.2  载荷长度 + 字段 p0/p1
        check("T2.2a CMD_ODOM 载荷 24 字节",
              len(pl) == 24, f"got {len(pl)} B")
        check("T2.2b CMD_ODOM p0 = position.x",
              abs(p0 - TEST_X) < 1e-4, f"p0={p0:.4f} expect={TEST_X}")
        check("T2.2c CMD_ODOM p1 = position.y",
              abs(p1 - TEST_Y) < 1e-4, f"p1={p1:.4f} expect={TEST_Y}")

        # T2.3  yaw 为角度制
        check("T2.3  CMD_ODOM yaw 为角度制（四元数已转换）",
              abs(yaw - TEST_YAW_DEG) < 0.02,
              f"yaw={yaw:.3f}° expect={TEST_YAW_DEG:.3f}°")

        # T2.4  校验和正确（FrameParser 能解析即表示校验正确）
        check("T2.4  CMD_ODOM 帧校验和正确（FrameParser 已验证）",
              True)
    else:
        for tag in ["T2.2a", "T2.2b", "T2.2c", "T2.3", "T2.4"]:
            skip(tag, "T2.1 未通过，跳过")

    # T2.5  上行帧率约 50 Hz
    drain()
    node.pub_odom()           # 写入 _latest_odom，定时器持续发送
    t0    = time.monotonic()
    count = 0
    while time.monotonic() - t0 < 1.0:
        raw = ser.read(256)
        if raw:
            for cmd, _ in parser.feed(raw):
                if cmd == CMD_ODOM:
                    count += 1
    check("T2.5  CMD_ODOM 帧率 40–60 Hz（1 秒内计数）",
          40 <= count <= 65, f"实际 {count} 帧/s")

    # T2.6  /target_depth(valid=True) → CMD_KFS + CMD_KFS_LATERAL_ERR
    if HAS_TARGET_DEPTH:
        drain()
        node.pub_target(class_id=3, cx=0.02, cy=-0.05, cz=0.65,
                        center_depth=0.65, in_range=True, valid=True)
        time.sleep(0.08)
        frames = read_frames(timeout=0.4)
        cmds   = [cmd for cmd, _ in frames]
        has_kfs = CMD_KFS in cmds
        has_lat = CMD_KFS_LATERAL_ERR in cmds
        check("T2.6a /target_depth(valid=True) → CMD_KFS(0x03) 帧发出",
              has_kfs, f"cmds={[hex(c) for c in cmds]}")
        check("T2.6b /target_depth(valid=True) → CMD_KFS_LATERAL_ERR(0x06) 帧发出",
              has_lat)

        # T2.7  CMD_KFS 载荷字段
        kfs_frames = [(cmd, pl) for cmd, pl in frames if cmd == CMD_KFS]
        if kfs_frames:
            _, kp = kfs_frames[0]
            num, det_id = struct.unpack("<BB", kp[:2])
            cx_r, cy_r, cz_r = struct.unpack("<3f", kp[2:14])
            check("T2.7a CMD_KFS num = 1",
                  num == 1, f"got {num}")
            check("T2.7b CMD_KFS class_id 正确",
                  det_id == 3, f"got {det_id}")
            check("T2.7c CMD_KFS z = center_depth",
                  abs(cz_r - 0.65) < 1e-4, f"z={cz_r:.4f} expect=0.65")
        else:
            for tag in ["T2.7a", "T2.7b", "T2.7c"]:
                skip(tag, "T2.6a 未通过，跳过")

        # T2.8  valid=False → 不发 CMD_KFS
        drain()
        node.pub_target(valid=False)
        time.sleep(0.15)
        frames2 = read_frames(timeout=0.3)
        kfs2    = [c for c, _ in frames2 if c == CMD_KFS]
        check("T2.8  /target_depth(valid=False) → 不发 CMD_KFS",
              len(kfs2) == 0, f"意外收到 {len(kfs2)} 帧")
    else:
        for tag in ["T2.6a", "T2.6b", "T2.7a", "T2.7b", "T2.7c", "T2.8"]:
            skip(tag, "vision_grasp_interfaces 未安装")


# ─── 入口 ────────────────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser(description="serial_bridge_node 自动化测试")
    ap.add_argument("--port", default="/tmp/vserial_tester",
                    help="测试端虚拟串口路径（默认 /tmp/vserial_tester）")
    ap.add_argument("--baud", type=int, default=115200)
    args = ap.parse_args()

    print("=" * 60)
    print("  serial_bridge_node 自动化测试")
    print(f"  测试端口: {args.port}  波特率: {args.baud}")
    print("=" * 60)

    # 依赖检查
    if not HAS_SERIAL:
        print(f"[{FAIL}] 未安装 pyserial（pip install pyserial）")
        sys.exit(1)
    if not HAS_ROS:
        print(f"[{FAIL}] rclpy 未找到，请先 source ROS2 环境")
        sys.exit(1)

    # 打开串口
    try:
        ser = pyserial.Serial(args.port, args.baud, timeout=0.05)
        print(f"[{INFO}] 串口已打开：{args.port}")
    except Exception as e:
        print(f"[{FAIL}] 串口打开失败：{e}")
        print(f"\n  请先在另一个终端运行：")
        print(f"    socat -d -d \\")
        print(f"      pty,raw,echo=0,link=/tmp/vserial_bridge \\")
        print(f"      pty,raw,echo=0,link=/tmp/vserial_tester")
        sys.exit(1)

    # 启动 ROS
    rclpy.init()
    node = TesterNode()
    spin_t = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_t.start()
    time.sleep(0.6)   # 等待订阅握手完成
    print(f"[{INFO}] ROS 节点已就绪")

    parser = FrameParser()

    try:
        run_tests(ser, node, parser)
    finally:
        ser.close()
        node.destroy_node()
        rclpy.shutdown()

    # ── 汇总 ─────────────────────────────────────────────────────────────────
    done    = [(n, ok) for n, ok in _results if ok is not None]
    skipped = [n for n, ok in _results if ok is None]
    passed  = sum(1 for _, ok in done if ok)
    failed  = sum(1 for _, ok in done if not ok)

    print("\n" + "=" * 60)
    print(f"  结果：{_G}{passed} 通过{_N}  {_R}{failed} 失败{_N}  "
          f"{_Y}{len(skipped)} 跳过{_N}  （共 {len(done)} 项）")
    if failed:
        print(f"  {_R}失败项：{_N}")
        for name, ok in done:
            if not ok:
                print(f"    · {name}")
    if skipped:
        print(f"  {_Y}跳过项：{_N} {', '.join(skipped)}")
    print("=" * 60)

    sys.exit(0 if failed == 0 else 1)


if __name__ == "__main__":
    main()

