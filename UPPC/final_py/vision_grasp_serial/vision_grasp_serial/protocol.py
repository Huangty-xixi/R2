# -*- coding: utf-8 -*-
"""
upper_pc_protocol.py  ——  上位机（NUC）串口通信协议
与下位机 upper_pc_protocol.h / upper_pc_protocol.c 完全对齐

帧格式（上行 / 下行统一）：
    ┌──────┬──────┬─────┬───────┬───────┬────────────────┬────────┐
    │ 0xA5 │ 0x5A │ CMD │ LEN_L │ LEN_H │  PAYLOAD[LEN]  │ CHKSUM │
    └──────┴──────┴─────┴───────┴───────┴────────────────┴────────┘
      SYNC1  SYNC2          └── uint16 小端 ──┘             1 字节

    CHKSUM = CMD ^ LEN_L ^ LEN_H ^ payload[0] ^ … ^ payload[n-1]
    与 C 代码 calc_chk(cmd, data, len) 完全一致。

━━━━━━━━ 上行 (NUC → STM32) ━━━━━━━━

  0x01  CMD_ODOM          里程计，全程持续上行
        载荷 24B = 6 × float32 LE，字段顺序严格对应 handle_odom 的偏移：
            data[0]  p0      FAST-LIO2 坐标原始值，下位机据场地映射为 robot_y
            data[4]  p1      FAST-LIO2 坐标原始值，下位机据红/蓝场决定 ±robot_x
            data[8]  z       高度 (m)
            data[12] roll    横滚角 (deg)
            data[16] pitch   俯仰角 (deg)
            data[20] yaw     航向角 (deg)，wrap±180 由下位机 wrap_deg_180 执行
        注：p0/p1 对应 FAST-LIO2 position.x / position.y，
            上位机不做坐标系映射，直接传原始值。
            角度必须转换为度（°）后填入。

  0x02  CMD_PATH          路径点（可选，路径规划时使用）
        载荷：num(u8) + [x(f32) y(f32)] × num，最多 16 个路径点

  0x03  CMD_KFS           摄像头目标检测结果，检测使能期间上行
        载荷：num(u8) + [id(u8) x(f32) y(f32) z(f32)] × num
            num     目标数量（本项目固定 1）
            id      物块类别 id（YOLO class_id）
            x y z   目标在相机坐标系下的三维坐标 (m)，来自反投影结果
        对应下位机 handle_kfs，下位机回调 cb_kfs 获取后进行抓取规划。

  0x05  CMD_ZONE_I_PATH   I 区路径（由上位机路径规划模块生成后下发）
        载荷：start_block(u8) end_block(u8) num_blocks(u8) [block_id(u8)] × num

  0x06  CMD_KFS_LATERAL_ERR  摄像头 KFS 相机坐标，用于下位机计算横向误差
        载荷 12B = 3 × float32 LE：
            x(f32)  KFS 在相机坐标系下的 X (m)
            y(f32)  KFS 在相机坐标系下的 Y (m)
            z(f32)  KFS 在相机坐标系下的 Z (m)，即中心深度 center_depth
        对应下位机 handle_kfs_lateral_err → camera_kfs_to_lateral_error。
        与 CMD_KFS 同时发送，互补：
            CMD_KFS 提供类别+位置供抓取规划
            CMD_KFS_LATERAL_ERR 提供下位机对准校正所需的横向误差

━━━━━━━━ 下行 (STM32 → NUC) ━━━━━━━━

  0x10  CMD_ACK            确认：acked_cmd(u8) + code(u8)，code 0=OK 1=ERR
  0x12  CMD_STATUS         机器人状态：state(u8)，见 RC_STATE_* 枚举
                            0 IDLE / 4 DONE → 可关摄像头
  0x13  CMD_ZONE_I_INFO    I 区 KFS 布局：num(u8) + [block_id(u8) kfs_type(u8)] × num
                            kfs_type: 1=R1_KFS 2=R2_KFS 3=FAKE
  0x14  CMD_DOCK_OK        R1 对接成功（空载荷）
  0x15  CMD_GO_ZONE_I      请求进入 I 区（空载荷）★ 触发开摄像头 ★
  0x20  CMD_DEBUG_HEADING  航向保持 PID 调试：6 × float32
                            yaw_ref_deg yaw_deg err_deg i_term output yaw_rate_dps
  0x21  CMD_DEBUG_NAV      导航到点调试：6 × float32
                            ex ey dist zone vy_fwd vw_str
"""

import struct
import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

# ────────────────────────── 帧头 ──────────────────────────
HEAD0 = 0xA5   # RC_SYNC1
HEAD1 = 0x5A   # RC_SYNC2

# ────────────────────────── 命令字 ──────────────────────────
# 上行（NUC → STM32）
CMD_ODOM              = 0x01   # RC_CMD_ODOM
CMD_PATH              = 0x02   # RC_CMD_PATH
CMD_KFS               = 0x03   # RC_CMD_KFS
CMD_ZONE_I_PATH       = 0x05   # RC_CMD_ZONE_I_PATH
CMD_KFS_LATERAL_ERR   = 0x06   # RC_CMD_KFS_LATERAL_ERR

# 下行（STM32 → NUC）
CMD_ACK               = 0x10   # RC_CMD_ACK
CMD_STATUS            = 0x12   # RC_CMD_STATUS
CMD_ZONE_I_INFO       = 0x13   # RC_CMD_ZONE_I_INFO
CMD_DOCK_OK           = 0x14   # RC_CMD_DOCK_OK
CMD_GO_ZONE_I         = 0x15   # RC_CMD_GO_ZONE_I
CMD_DEBUG_HEADING     = 0x20   # RC_CMD_DEBUG_HEADING_HOLD
CMD_DEBUG_NAV         = 0x21   # RC_CMD_DEBUG_NAV_GOTO

# ────────────────────────── 枚举 ──────────────────────────
# 下位机状态 rc_state_t
RC_STATE_IDLE       = 0
RC_STATE_MOVING     = 1
RC_STATE_AT_TARGET  = 2
RC_STATE_GRABBING   = 3
RC_STATE_DONE       = 4
RC_STATE_ERROR      = 5

# I 区 KFS 类型
KFS_TYPE_R1   = 1
KFS_TYPE_R2   = 2
KFS_TYPE_FAKE = 3

# ACK 返回码
ACK_OK  = 0
ACK_ERR = 1

# 帧约束（与 C 宏对齐）
RC_FRAME_HEADER_SIZE  = 5    # SYNC1 SYNC2 CMD LEN_L LEN_H
RC_FRAME_MAX_PAYLOAD  = 64
RC_FRAME_MAX_SIZE     = RC_FRAME_HEADER_SIZE + RC_FRAME_MAX_PAYLOAD + 1


# ══════════════════════════════════════════════════════════════
# 校验和（与 C 代码 calc_chk 完全一致）
# ══════════════════════════════════════════════════════════════
def calc_checksum(cmd: int, payload: bytes) -> int:
    """1 字节 XOR 校验：CMD ^ LEN_L ^ LEN_H ^ 每个载荷字节。"""
    n   = len(payload)
    chk = cmd ^ (n & 0xFF) ^ ((n >> 8) & 0xFF)
    for b in payload:
        chk ^= b
    return chk & 0xFF


# ══════════════════════════════════════════════════════════════
# 通用打帧（上行下行均调用）
# ══════════════════════════════════════════════════════════════
def build_frame(cmd: int, payload: bytes = b"") -> bytes:
    """打包一帧，LEN 2 字节小端，CHKSUM 1 字节 XOR。
    
    帧结构：SYNC1 SYNC2 CMD LEN_L LEN_H PAYLOAD[LEN] CHKSUM
    与 C 代码 send_frame 完全一致。
    """
    if len(payload) > RC_FRAME_MAX_PAYLOAD:
        raise ValueError(
            f"payload 超过最大长度 {RC_FRAME_MAX_PAYLOAD} 字节，"
            f"实际 {len(payload)} 字节")
    n   = len(payload)
    chk = calc_checksum(cmd, payload)
    return bytes([HEAD0, HEAD1, cmd, n & 0xFF, (n >> 8) & 0xFF]) + payload + bytes([chk])


# ══════════════════════════════════════════════════════════════
# 上行载荷编码
# ══════════════════════════════════════════════════════════════

def encode_odom(p0: float, p1: float, z: float,
                roll_deg: float, pitch_deg: float, yaw_deg: float) -> bytes:
    """打包 CMD_ODOM 帧（24 字节 = 6 × float32 LE）。

    字段顺序严格对应下位机 handle_odom 读取偏移：
        data[0]  p0      → 下位机 robot_y（+ 固定偏移，下位机内部处理）
        data[4]  p1      → 下位机 ±robot_x（符号取决于红/蓝场，下位机内部处理）
        data[8]  z       → 高度 (m)
        data[12] roll    → 横滚角 (deg)
        data[16] pitch   → 俯仰角 (deg)
        data[20] yaw     → 航向角 (deg)，wrap_deg_180 由下位机执行

    调用方应传入：
        p0 = FAST-LIO2 position.x
        p1 = FAST-LIO2 position.y
        roll_deg / pitch_deg / yaw_deg 已由四元数转换为角度制
    """
    payload = struct.pack("<6f", p0, p1, z, roll_deg, pitch_deg, yaw_deg)
    return build_frame(CMD_ODOM, payload)


def encode_path(waypoints: List[Tuple[float, float]]) -> bytes:
    """打包 CMD_PATH 帧。
    
    载荷：num(u8) + [x(f32) y(f32)] × num，最多 16 个路径点。
    对应下位机 handle_path。
    """
    if len(waypoints) > 16:
        waypoints = waypoints[:16]
    n   = len(waypoints)
    payload = bytes([n])
    for x, y in waypoints:
        payload += struct.pack("<2f", x, y)
    return build_frame(CMD_PATH, payload)


def encode_kfs(detections: List[Tuple[int, float, float, float]]) -> bytes:
    """打包 CMD_KFS 帧。
    
    detections: [(id, x, y, z), ...]，最多 8 个目标
        id    : 物块类别编号（YOLO class_id）
        x y z : 目标在相机坐标系下的三维坐标 (m)，来自反投影
    载荷：num(u8) + [id(u8) x(f32) y(f32) z(f32)] × num
    对应下位机 handle_kfs。
    
    单目标（最常用）：
        encode_kfs([(class_id, cam_x, cam_y, cam_z)])
    """
    if len(detections) > 8:
        detections = detections[:8]
    n   = len(detections)
    payload = bytes([n])
    for det_id, x, y, z in detections:
        payload += struct.pack("<B3f", det_id & 0xFF, x, y, z)
    return build_frame(CMD_KFS, payload)


def encode_kfs_single(class_id: int,
                      cam_x: float, cam_y: float, cam_z: float) -> bytes:
    """encode_kfs 的单目标快捷版（最常用场景）。"""
    return encode_kfs([(class_id, cam_x, cam_y, cam_z)])


def encode_kfs_lateral_err(cam_x: float, cam_y: float, cam_z: float) -> bytes:
    """打包 CMD_KFS_LATERAL_ERR 帧（12 字节 = 3 × float32 LE）。
    
    cam_x / cam_y / cam_z：KFS 目标在相机坐标系下的三维坐标 (m)
        cam_z = center_depth（来自分割掩膜中心稳健中值）
    对应下位机 handle_kfs_lateral_err → camera_kfs_to_lateral_error。
    下位机通过 rc_get_kfs_lateral_err_m() 读取横向误差用于对准校正。
    """
    payload = struct.pack("<3f", cam_x, cam_y, cam_z)
    return build_frame(CMD_KFS_LATERAL_ERR, payload)


def encode_zone_i_path(start_block: int, end_block: int,
                       block_ids: List[int]) -> bytes:
    """打包 CMD_ZONE_I_PATH 帧。
    
    载荷：start_block(u8) end_block(u8) num_blocks(u8) [block_id(u8)] × num
    对应下位机 handle_zone_i_path。最多 32 个 block_id。
    """
    if len(block_ids) > 32:
        block_ids = block_ids[:32]
    n   = len(block_ids)
    payload = bytes([start_block & 0xFF, end_block & 0xFF, n]) + bytes(block_ids)
    return build_frame(CMD_ZONE_I_PATH, payload)


# ══════════════════════════════════════════════════════════════
# 下行载荷解码（辅助函数，供 FrameParser 分发后使用）
# ══════════════════════════════════════════════════════════════

@dataclass
class AckPayload:
    acked_cmd: int   # 被确认的上行 CMD
    code: int        # 0=OK, 1=ERR

def decode_ack(payload: bytes) -> Optional[AckPayload]:
    """解码 CMD_ACK（0x10）载荷。"""
    if len(payload) < 2:
        return None
    return AckPayload(acked_cmd=payload[0], code=payload[1])


@dataclass
class ZoneIKfs:
    block_id: int
    kfs_type: int   # 1=R1_KFS, 2=R2_KFS, 3=FAKE

@dataclass
class ZoneIInfoPayload:
    detections: List[ZoneIKfs] = field(default_factory=list)

def decode_zone_i_info(payload: bytes) -> Optional[ZoneIInfoPayload]:
    """解码 CMD_ZONE_I_INFO（0x13）载荷。"""
    if len(payload) < 1:
        return None
    n   = payload[0]
    result = ZoneIInfoPayload()
    pos = 1
    for _ in range(n):
        if pos + 2 > len(payload):
            break
        result.detections.append(
            ZoneIKfs(block_id=payload[pos], kfs_type=payload[pos + 1]))
        pos += 2
    return result


@dataclass
class DebugHeadingPayload:
    yaw_ref_deg: float
    yaw_deg: float
    err_deg: float
    i_term: float
    output: float
    yaw_rate_dps: float

def decode_debug_heading(payload: bytes) -> Optional[DebugHeadingPayload]:
    """解码 CMD_DEBUG_HEADING_HOLD（0x20）载荷（6 × float32）。"""
    if len(payload) < 24:
        return None
    v = struct.unpack("<6f", payload[:24])
    return DebugHeadingPayload(*v)


@dataclass
class DebugNavPayload:
    ex: float
    ey: float
    dist: float
    zone: float
    vy_fwd: float
    vw_str: float

def decode_debug_nav(payload: bytes) -> Optional[DebugNavPayload]:
    """解码 CMD_DEBUG_NAV_GOTO（0x21）载荷（6 × float32）。"""
    if len(payload) < 24:
        return None
    v = struct.unpack("<6f", payload[:24])
    return DebugNavPayload(*v)


# ══════════════════════════════════════════════════════════════
# 坐标/角度转换工具
# ══════════════════════════════════════════════════════════════

def quat_to_euler_deg(qx: float, qy: float,
                      qz: float, qw: float) -> Tuple[float, float, float]:
    """四元数 → (roll, pitch, yaw) 角度制 (deg)。
    
    用于将 nav_msgs/Odometry 的四元数转换为下位机期望的欧拉角（度）。
    wrap±180 由下位机 wrap_deg_180 执行，上位机无需处理。
    """
    # roll（绕 x 轴）
    sinr = 2.0 * (qw * qx + qy * qz)
    cosr = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.degrees(math.atan2(sinr, cosr))

    # pitch（绕 y 轴）
    sinp = max(-1.0, min(1.0, 2.0 * (qw * qy - qz * qx)))
    pitch = math.degrees(math.asin(sinp))

    # yaw（绕 z 轴）
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.degrees(math.atan2(siny, cosy))

    return roll, pitch, yaw


# ══════════════════════════════════════════════════════════════
# 通用增量式帧解析器（与下位机 rc_feed_byte 逻辑对称）
# ══════════════════════════════════════════════════════════════

@dataclass
class Frame:
    """解析出的完整帧。"""
    cmd:     int
    payload: bytes


class FrameParser:
    """增量式帧解析器，适配下位机帧格式。

    特性：
    - LEN 2 字节小端，CHKSUM 1 字节 XOR
    - 同步头丢失或校验失败时自动滑动 1 字节重新搜索，抗电磁干扰
    - 可任意分段喂入字节流（每字节、每包、每行均可）

    用法：
        parser = FrameParser()
        for frame in parser.feed(raw_bytes):
            dispatch(frame.cmd, frame.payload)
    """

    def __init__(self, max_buffer: int = 4096) -> None:
        self._buf        = bytearray()
        self._max_buffer = max_buffer

    def feed(self, data: bytes) -> List[Frame]:
        """喂入新字节，返回本次完成解析的帧列表（可能为空）。"""
        self._buf.extend(data)
        if len(self._buf) > self._max_buffer:
            del self._buf[:-self._max_buffer]

        frames: List[Frame] = []
        while True:
            frame, consumed = self._try_parse_one()
            if consumed == 0:
                break
            del self._buf[:consumed]
            if frame is not None:
                frames.append(frame)
        return frames

    def _try_parse_one(self) -> Tuple[Optional[Frame], int]:
        """尝试从缓冲区头部解析一帧。

        返回值：
            (Frame, n)  校验通过，消耗 n 字节
            (None,  1)  头部不合法/校验失败，跳过 1 字节
            (None,  0)  数据不足，等待更多字节
        """
        buf = self._buf

        if len(buf) < 2:
            return None, 0

        # 同步头检查
        if not (buf[0] == HEAD0 and buf[1] == HEAD1):
            return None, 1      # 跳 1 字节重新搜索

        # 头部至少需要 5 字节
        if len(buf) < RC_FRAME_HEADER_SIZE:
            return None, 0

        cmd    = buf[2]
        length = buf[3] | (buf[4] << 8)    # LEN 小端 uint16

        if length > RC_FRAME_MAX_PAYLOAD:
            return None, 1      # 载荷长度异常，跳字节重同步

        total = RC_FRAME_HEADER_SIZE + length + 1   # 头+载荷+校验
        if len(buf) < total:
            return None, 0      # 帧未收全

        payload  = bytes(buf[RC_FRAME_HEADER_SIZE : RC_FRAME_HEADER_SIZE + length])
        recv_chk = buf[total - 1]
        calc_chk = calc_checksum(cmd, payload)

        if recv_chk != calc_chk:
            return None, 1      # 校验失败，跳字节

        return Frame(cmd=cmd, payload=payload), total