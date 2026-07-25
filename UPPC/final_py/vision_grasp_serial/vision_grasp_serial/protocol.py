# -*- coding: utf-8 -*-
"""
protocol.py  ——  上位机（NUC）串口通信协议
与下位机 upper_pc_protocol.h / upper_pc_protocol.c 完全对齐

帧格式（上行 / 下行统一）：
    ┌──────┬──────┬─────┬───────┬───────┬────────────────┬────────┐
    │ 0xA5 │ 0x5A │ CMD │ LEN_L │ LEN_H │  PAYLOAD[LEN]  │ CHKSUM │
    └──────┴──────┴─────┴───────┴───────┴────────────────┴────────┘
      SYNC1  SYNC2          └── uint16 小端 ──┘             1 字节

    CHKSUM = CMD ^ LEN_L ^ LEN_H ^ payload[0] ^ … ^ payload[n-1]
    与 C 代码 calc_chk(cmd, data, len) 完全一致。

━━━━━━━━ 上行 (NUC → STM32) ━━━━━━━━
  0x01 CMD_ODOM        里程计，全程持续上行（50Hz）
  0x02 CMD_PATH        路径点（可选）
  0x03 CMD_KFS         摄像头目标检测结果
  0x05 CMD_ZONE_I_PATH  I 区路径
  0x06 CMD_KFS_LATERAL_ERR 摄像头KFS横向误差坐标

━━━━━━━━ 下行 (STM32 → NUC) ━━━━━━━━
  0x10 CMD_ACK             确认帧
  0x12 CMD_STATUS          机器人状态
  0x13 CMD_ZONE_I_INFO     I区KFS布局
  0x14 CMD_DOCK_OK         R1对接成功
  0x15 CMD_GO_ZONE_I       请求入I区（触发开摄像头）
  0x20 CMD_DEBUG_HEADING   航向保持PID调试
  0x21 CMD_DEBUG_NAV       导航到点调试
"""

import struct                    # struct.pack('<f', 3.14) 把Python float转成4字节小端序
                                  # struct.unpack('<6f', 24bytes) 把24字节拆成6个float
                                  # '<' = 小端序（little-endian），和STM32 ARM一致
import math                       # math.degrees() 弧度转度，math.atan2() 四元数转欧拉角
from dataclasses import dataclass, field  # @dataclass 自动生成 __init__（= C 的结构体自动初始化）
                                           # 让你写结构体时不用手写构造函数
from typing import List, Optional, Tuple   # 类型注解（文档用，运行时不影响）

# ────────────────────────── 帧头（和 STM32 的 RC_SYNC1/RC_SYNC2 一致） ──────────────────────────
HEAD0 = 0xA5   # 帧头第1字节
HEAD1 = 0x5A   # 帧头第2字节

# ────────────────────────── 命令字（和 STM32 的 rc_cmd_t 枚举一致） ──────────────────────────
# 上行（NUC → STM32）
CMD_ODOM              = 0x01   # 里程计：6个float
CMD_PATH              = 0x02   # 路径点：num + [x,y]×N
CMD_KFS               = 0x03   # KFS检测：num + [id,x,y,z]×N
CMD_ZONE_I_PATH       = 0x05   # I区路径
CMD_KFS_LATERAL_ERR   = 0x06   # KFS横向误差：3个float

# 下行（STM32 → NUC）
CMD_ACK               = 0x10   # 确认帧
CMD_STATUS            = 0x12   # 状态上报
CMD_ZONE_I_INFO       = 0x13   # I区KFS布局
CMD_DOCK_OK           = 0x14   # R1对接成功
CMD_GO_ZONE_I         = 0x15   # 请求入I区
CMD_DEBUG_HEADING     = 0x20   # 航向PID调试
CMD_DEBUG_NAV         = 0x21   # 导航PID调试

# ────────────────────────── 枚举（和 STM32 的 typedef enum 一致） ──────────────────────────
RC_STATE_IDLE       = 0  # 空闲
RC_STATE_MOVING     = 1  # 移动中
RC_STATE_AT_TARGET  = 2  # 到达目标
RC_STATE_GRABBING   = 3  # 抓取中
RC_STATE_DONE       = 4  # 完成
RC_STATE_ERROR      = 5  # 错误

KFS_TYPE_R1   = 1  # R1 的 KFS
KFS_TYPE_R2   = 2  # R2 的 KFS
KFS_TYPE_FAKE = 3  # 假的 KFS（障碍物）

ACK_OK  = 0   # 确认成功
ACK_ERR = 1   # 确认失败

# 帧约束（和 C 宏对齐——STM32 里 #define 的值）
RC_FRAME_HEADER_SIZE  = 5     # 帧头 5 字节：SYNC1 + SYNC2 + CMD + LEN_L + LEN_H
RC_FRAME_MAX_PAYLOAD  = 64    # 载荷最大 64 字节
RC_FRAME_MAX_SIZE     = RC_FRAME_HEADER_SIZE + RC_FRAME_MAX_PAYLOAD + 1  # 头部+载荷+校验


# ══════════════════════════════════════════════════════════════
# 校验和（与 C 代码 calc_chk 完全一致）
# ══════════════════════════════════════════════════════════════

def calc_checksum(cmd: int, payload: bytes) -> int:
    """
    1 字节 XOR 校验：CMD ^ LEN_L ^ LEN_H ^ 每个载荷字节。
    对应 STM32 的 static uint8_t calc_chk(uint8_t cmd, const uint8_t *data, uint16_t len)

    参数类型注解（Python 3.6+ 语法，只用于文档和IDE提示，不强制类型）：
      cmd: int       → 命令码
      payload: bytes → 载荷字节串
      → int          → 返回校验值（0-255）
    """
    n   = len(payload)                           # payload 的长度
    chk = cmd ^ (n & 0xFF) ^ ((n >> 8) & 0xFF)   # CMD ^ LEN_L ^ LEN_H
                                                  # ^ = Python 的 XOR 运算符（和 C 的 ^ 一样）
                                                  # & 0xFF = 只取低 8 位
    for b in payload:                             # Python 的 for-in 循环（= C 的 for(i=0;i<len;i++)）
        chk ^= b                                  # XOR 每个字节
    return chk & 0xFF                             # 截断到 8 位


# ══════════════════════════════════════════════════════════════
# 通用打帧（上行下行均调用）
# ══════════════════════════════════════════════════════════════

def build_frame(cmd: int, payload: bytes = b"") -> bytes:
    """
    打包一帧：SYNC1 SYNC2 CMD LEN_L LEN_H PAYLOAD CHKSUM
    对应 STM32 的 send_frame()

    payload: bytes = b""   → 参数默认值。b"" 是空字节串（= C 的空数组 {}）
    """
    if len(payload) > RC_FRAME_MAX_PAYLOAD:     # 载荷太长 → 抛异常（宁可崩溃也不发错误数据）
        raise ValueError(                        # raise = Python 的 throw exception
            f"payload 超过最大长度 {RC_FRAME_MAX_PAYLOAD} 字节，"
            f"实际 {len(payload)} 字节")
    n   = len(payload)
    chk = calc_checksum(cmd, payload)
    # bytes([...]) = 把整数列表转成字节数组（和 C 的 uint8_t arr[] = {0xA5,0x5A,...} 一样）
    # n & 0xFF = 低 8 位（LEN_L）
    # (n >> 8) & 0xFF = 高 8 位（LEN_H）
    return bytes([HEAD0, HEAD1, cmd, n & 0xFF, (n >> 8) & 0xFF]) + payload + bytes([chk])


# ══════════════════════════════════════════════════════════════
# 上行载荷编码（Python 数据 → 串口字节帧）
# ══════════════════════════════════════════════════════════════

def encode_odom(p0: float, p1: float, z: float,
                roll_deg: float, pitch_deg: float, yaw_deg: float) -> bytes:
    """
    打包 CMD_ODOM 帧（24 字节 = 6 × float32 LE）

    字段顺序严格对应下位机 handle_odom 读取偏移：
        data[0]  p0      → 下位机 robot_y
        data[4]  p1      → 下位机 ±robot_x
        data[8]  z       → 高度
        data[12] roll    → 横滚角
        data[16] pitch   → 俯仰角
        data[20] yaw     → 航向角

    struct.pack('<6f', ...) = 把 6 个 Python float 按小端序打包成 24 字节
    '<' = 小端序（little-endian），和 STM32 ARM Cortex-M7 一致
    '6f' = 6 个 float（每个 float 占 4 字节）
    """
    payload = struct.pack("<6f", p0, p1, z, roll_deg, pitch_deg, yaw_deg)
    return build_frame(CMD_ODOM, payload)


def encode_path(waypoints: List[Tuple[float, float]]) -> bytes:
    """
    打包 CMD_PATH 帧。
    waypoints: [(x1,y1), (x2,y2), ...]，最多 16 个
    载荷：num(u8) + [x(f32) y(f32)] × num
    """
    if len(waypoints) > 16:
        waypoints = waypoints[:16]               # Python 切片：取前 16 个元素
    n   = len(waypoints)
    payload = bytes([n])                         # bytes([n]) = 单字节数组，值为 n
    for x, y in waypoints:                       # Python 元组解包：x,y = (1.0, 2.0)
        payload += struct.pack("<2f", x, y)      # 2 个 float = 8 字节
    return build_frame(CMD_PATH, payload)


def encode_kfs(detections: List[Tuple[int, float, float, float]]) -> bytes:
    """
    打包 CMD_KFS 帧。
    detections: [(id, x, y, z), ...]，最多 8 个
    载荷：num(u8) + [id(u8) x(f32) y(f32) z(f32)] × num
    """
    if len(detections) > 8:
        detections = detections[:8]
    n   = len(detections)
    payload = bytes([n])
    for det_id, x, y, z in detections:
        # '<B3f' = 1 个 unsigned char (u8) + 3 个 float
        payload += struct.pack("<B3f", det_id & 0xFF, x, y, z)
    return build_frame(CMD_KFS, payload)


def encode_kfs_single(class_id: int, cam_x: float, cam_y: float, cam_z: float) -> bytes:
    """encode_kfs 的单目标快捷版（最常用——摄像头一次只检测一个目标）"""
    return encode_kfs([(class_id, cam_x, cam_y, cam_z)])


def encode_kfs_lateral_err(cam_x: float, cam_y: float, cam_z: float) -> bytes:
    """
    打包 CMD_KFS_LATERAL_ERR 帧（12 字节 = 3 × float32 LE）。
    下位机用这个数据和 CMD_KFS(0x03) 配合做对准校正。
    """
    payload = struct.pack("<3f", cam_x, cam_y, cam_z)
    return build_frame(CMD_KFS_LATERAL_ERR, payload)


def encode_zone_i_path(start_block: int, end_block: int,
                       block_ids: List[int]) -> bytes:
    """打包 CMD_ZONE_I_PATH 帧：startBlock + endBlock + N + [blockId]*N"""
    if len(block_ids) > 32:
        block_ids = block_ids[:32]
    n   = len(block_ids)
    # bytes([a,b,c]) + bytes(list) = 拼接字节数组
    payload = bytes([start_block & 0xFF, end_block & 0xFF, n]) + bytes(block_ids)
    return build_frame(CMD_ZONE_I_PATH, payload)


# ══════════════════════════════════════════════════════════════
# 下行载荷解码（串口字节帧 → Python 数据对象）
# ══════════════════════════════════════════════════════════════

@dataclass                              # Python 装饰器：自动生成 __init__、__repr__ 等方法
                                        # 等价于 C 里写了一个 typedef struct { ... } 并手写初始化
class AckPayload:
    """ACK 载荷（0x10）"""
    acked_cmd: int   # 被确认的上行 CMD
    code: int        # 0=OK, 1=ERR

def decode_ack(payload: bytes) -> Optional[AckPayload]:
    """解码 CMD_ACK 载荷。Optional[AckPayload] = 可能返回 None"""
    if len(payload) < 2: return None                   # 数据不完整 → 返回 None（= C 的 NULL）
    return AckPayload(acked_cmd=payload[0], code=payload[1])  # 创建结构体对象


@dataclass
class ZoneIKfs:
    """I 区单个 KFS 信息"""
    block_id: int
    kfs_type: int   # 1=R1_KFS, 2=R2_KFS, 3=FAKE

@dataclass
class ZoneIInfoPayload:
    """I 区 KFS 布局载荷（0x13）"""
    detections: List[ZoneIKfs] = field(default_factory=list)  # 默认值 = 空列表
                                                                # field(default_factory=list) = Python 的"每次创建新对象时给不同的空列表"

def decode_zone_i_info(payload: bytes) -> Optional[ZoneIInfoPayload]:
    """解码 CMD_ZONE_I_INFO 载荷。逐字节解析 num + [block_id, kfs_type] × num"""
    if len(payload) < 1: return None
    n   = payload[0]                        # 第一个字节 = 检测结果数量
    result = ZoneIInfoPayload()
    pos = 1                                 # 从第 2 个字节开始读
    for _ in range(n):                      # Python 的 for i in range(n) = C 的 for(i=0;i<n;i++)
                                             # _ = 用不到的循环变量（约定俗成用下划线表示"我不需要这个值"）
        if pos + 2 > len(payload): break    # 防止越界
        result.detections.append(           # list.append(item) = C 的 arr[i] = item; i++
            ZoneIKfs(block_id=payload[pos], kfs_type=payload[pos + 1]))
        pos += 2
    return result


@dataclass
class DebugHeadingPayload:
    """航向保持PID调试数据（0x20）—— 6个float"""
    yaw_ref_deg: float     # 参考航向角
    yaw_deg: float         # 当前航向角
    err_deg: float         # 误差
    i_term: float          # 积分项
    output: float          # PID输出
    yaw_rate_dps: float    # 角速度 deg/s

def decode_debug_heading(payload: bytes) -> Optional[DebugHeadingPayload]:
    """解码 CMD_DEBUG_HEADING_HOLD 载荷（24字节 = 6 × float32）"""
    if len(payload) < 24: return None
    v = struct.unpack("<6f", payload[:24])   # unpack = pack 的反操作：24字节 → 6个float元组
    return DebugHeadingPayload(*v)            # *v = 元组展开（= C 里把数组元素逐个传给函数参数）


@dataclass
class DebugNavPayload:
    """导航到点调试数据（0x21）—— 6个float"""
    ex: float       # X 方向误差
    ey: float       # Y 方向误差
    dist: float     # 到目标距离
    zone: float     # 远近分区（0=远 1=近）
    vy_fwd: float   # 前后速度指令
    vw_str: float   # 左右速度指令

def decode_debug_nav(payload: bytes) -> Optional[DebugNavPayload]:
    """解码 CMD_DEBUG_NAV_GOTO 载荷（24字节 = 6 × float32）"""
    if len(payload) < 24: return None
    v = struct.unpack("<6f", payload[:24])
    return DebugNavPayload(*v)


# ══════════════════════════════════════════════════════════════
# 坐标/角度转换工具
# ══════════════════════════════════════════════════════════════

def quat_to_euler_deg(qx: float, qy: float,
                      qz: float, qw: float) -> Tuple[float, float, float]:
    """
    四元数 → (roll, pitch, yaw) 角度制 (deg)
    用于把 nav_msgs/Odometry 的四元数转成 STM32 期望的欧拉角（度）
    math.atan2() = C 的 atan2f()
    math.degrees() = 弧度转度（×180/π）
    """
    sinr = 2.0 * (qw * qx + qy * qz)                    # 横滚角的正弦
    cosr = 1.0 - 2.0 * (qx * qx + qy * qy)              # 横滚角的余弦
    roll = math.degrees(math.atan2(sinr, cosr))          # arctan → 度

    sinp = max(-1.0, min(1.0, 2.0 * (qw * qy - qz * qx))) # 俯仰角的正弦（钳位到 [-1,1] 防浮点越界）
    pitch = math.degrees(math.asin(sinp))                   # arcsin → 度

    siny = 2.0 * (qw * qz + qx * qy)                       # 航向角的正弦
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)                 # 航向角的余弦
    yaw = math.degrees(math.atan2(siny, cosy))              # arctan → 度

    return roll, pitch, yaw       # Python 一行返回多个值（= C 的用指针参数或结构体返回）


# ══════════════════════════════════════════════════════════════
# 增量式帧解析器（与下位机 rc_feed_byte 逻辑对称）
# ══════════════════════════════════════════════════════════════

@dataclass
class Frame:
    """解析出的完整一帧（只含 cmd 和 payload，头部已剥离）"""
    cmd:     int     # 命令码
    payload: bytes   # 载荷字节


class FrameParser:
    """
    增量式帧解析器 — 对应 STM32 的 rc_feed_byte 状态机

    Python 类（class）等效 C 的结构体 + 一组操作函数。
    self = C 里传的那个结构体指针（自动传，不用手写）。

    用法：
        parser = FrameParser()
        for frame in parser.feed(raw_bytes):  # feed() 返回一个生成器（yield），每拼出一帧就 yield 一帧
            dispatch(frame.cmd, frame.payload)
    """

    def __init__(self, max_buffer: int = 4096) -> None:
        self._buf        = bytearray()       # bytearray = 可变的字节数组（= C 的 uint8_t buf[]，可以 append）
        self._max_buffer = max_buffer        # 缓冲区最大长度（防内存泄漏）

    def feed(self, data: bytes) -> List[Frame]:
        """
        喂入新字节，返回本次完成解析的帧列表（可能为空）。
        可以任意分段喂入——每字节、每包、每行都行。
        """
        self._buf.extend(data)               # bytearray.extend = 在数组末尾追加新字节
        if len(self._buf) > self._max_buffer:
            del self._buf[:-self._max_buffer] # 超了 → 删掉最老的字节（滑动窗口）

        frames: List[Frame] = []
        while True:
            frame, consumed = self._try_parse_one()  # 尝试从头部解析一帧
            if consumed == 0:                        # 数据不够 → 等下一次 feed
                break
            del self._buf[:consumed]                 # 删掉已消费的字节（无论解析成功还是跳过的）
            if frame is not None:                    # Python 的 is not None = C 的 != NULL
                frames.append(frame)                 # list.append(item) = 往列表末尾加一个元素
        return frames

    def _try_parse_one(self) -> Tuple[Optional[Frame], int]:
        """
        尝试从缓冲区头部解析一帧。

        返回值（Python 可以返回两个值，用 Tuple）：
            (Frame, n)  → 解析成功，消耗 n 字节
            (None, 1)   → 头部不合法/校验失败，跳过 1 字节后重试
            (None, 0)   → 数据不足，等下次喂字节
        """
        buf = self._buf

        # 至少需要 2 字节才能检查同步头
        if len(buf) < 2:
            return None, 0

        # 找同步头 0xA5 0x5A
        if not (buf[0] == HEAD0 and buf[1] == HEAD1):
            return None, 1       # 不是同步头 → 滑动 1 字节

        # 需要完整头部（5字节）才能读长度
        if len(buf) < RC_FRAME_HEADER_SIZE:
            return None, 0       # 数据不够 → 等

        cmd    = buf[2]                          # 第 3 字节 = CMD
        length = buf[3] | (buf[4] << 8)         # 第 4-5 字节 = LEN（小端 uint16）
                                                  # Python 没有 uint16，用 int 的位运算模拟

        if length > RC_FRAME_MAX_PAYLOAD:        # 载荷长度异常（可能是干扰字节被误判为头）
            return None, 1

        total = RC_FRAME_HEADER_SIZE + length + 1  # 完整帧 = 头5字节 + payload 字节 + 校验1字节
        if len(buf) < total:
            return None, 0                       # 帧还没收全 → 等

        # 提取 payload 和校验字节
        payload  = bytes(buf[RC_FRAME_HEADER_SIZE : RC_FRAME_HEADER_SIZE + length])
        recv_chk = buf[total - 1]                # 收到的校验值（帧尾最后一字节）
        calc_chk = calc_checksum(cmd, payload)    # 重新计算校验值

        if recv_chk != calc_chk:                 # 校验不通过 → 这一帧是垃圾数据
            return None, 1

        # 校验通过 → 返回完整帧
        return Frame(cmd=cmd, payload=payload), total
