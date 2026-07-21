# -*- coding: utf-8 -*-
"""
自定义串口通讯协议 (NUC 上位机 <-> 下位机)  ——  v3

============================================================================
通信方向与内容
============================================================================

上行与下行使用完全相同的帧结构：

    +--------+--------+--------+--------+----------------+-----------+
    | HEAD0  | HEAD1  |  CMD   |  LEN   |  PAYLOAD[LEN]  | CRC16(LE) |
    | 0xAA   | 0x55   | 1 byte | 1 byte |   LEN bytes    |  2 bytes  |
    +--------+--------+--------+--------+----------------+-----------+

    CRC16 校验范围：CMD + LEN + PAYLOAD，小端写入，MODBUS 多项式 0xA001。
    字节序：所有多字节数值采用小端 (little-endian)。

-------- 上行 (上位机 NUC -> 下位机)：CMD 0x01 / 0x02 --------

    0x01  CMD_ODOM
          雷达里程计数据，全程持续上行。
          载荷 24 字节，格式 "<6f"：
              x(f32)   平面位置 X   (m)
              y(f32)   平面位置 Y   (m)
              yaw(f32) 航向角        (rad)
              vx(f32)  线速度 X      (m/s)
              vy(f32)  线速度 Y      (m/s)
              omega(f32) 角速度      (rad/s)

    0x02  CMD_DEPTH
          摄像头目标中心深度信息，检测使能期间持续上行。
          载荷 18 字节，格式 "<B4fB"：
              class_id(u8)      物块类别 id
              center_depth(f32) 目标中心深度 (m)，取自分割掩膜中心，非包围框均值
              gx(f32)           目标中心在夹爪坐标系下的 X 坐标 (m)
              gy(f32)           目标中心在夹爪坐标系下的 Y 坐标 (m)
              gz(f32)           目标中心在夹爪坐标系下的 Z 坐标 (m)
              in_range(u8)      是否进入合理夹取范围 (1=是, 0=否)；下位机据此判到达

-------- 下行 (下位机 -> 上位机 NUC)：CMD 0x10 / 0x11 --------

    0x10  CMD_CAM_ON
          开摄像头信号，载荷 0 字节。
          上位机收到后使能摄像头检测节点，开始发布 CMD_DEPTH。

    0x11  CMD_CAM_OFF
          关摄像头信号，载荷 0 字节。
          上位机收到后失能摄像头检测节点，停止发布 CMD_DEPTH。

双方均用同一个 FrameParser 解析对方发来的帧，统一 CRC 校验，
抵抗串口电磁干扰，防止噪声误触发。
============================================================================
"""

import struct
from dataclasses import dataclass
from typing import List, Optional, Tuple

# ---- 帧头（上行与下行共用） ----
HEAD0 = 0xAA
HEAD1 = 0x55

# ---- 上行命令字 (上位机 -> 下位机) ----
CMD_ODOM  = 0x01    # 雷达里程计
CMD_DEPTH = 0x02    # 摄像头目标中心深度信息

# ---- 下行命令字 (下位机 -> 上位机) ----
CMD_CAM_ON  = 0x10  # 开摄像头
CMD_CAM_OFF = 0x11  # 关摄像头


# ==========================================================================
# CRC16 校验（MODBUS，多项式 0xA001，初值 0xFFFF）
# ==========================================================================
def crc16_modbus(data: bytes) -> int:
    """计算 MODBUS CRC16。校验范围由调用方指定切片，此函数不假设范围。"""
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


# ==========================================================================
# 通用打帧（上行与下行均调用此函数）
# ==========================================================================
def build_frame(cmd: int, payload: bytes = b"") -> bytes:
    """把一条命令打包成完整帧，上行下行通用。payload 长度须 <= 255。

    帧结构：HEAD0 HEAD1 CMD LEN PAYLOAD[LEN] CRC16(小端)
    CRC 校验范围：CMD + LEN + PAYLOAD
    """
    if len(payload) > 255:
        raise ValueError("payload 长度不能超过 255 字节")
    body = bytes([cmd, len(payload)]) + payload   # CMD + LEN + PAYLOAD
    crc  = crc16_modbus(body)                     # 校验范围: body (不含帧头)
    return bytes([HEAD0, HEAD1]) + body + struct.pack("<H", crc)


# ==========================================================================
# 上行载荷编码（上位机调用，向下位机发送）
# ==========================================================================
def encode_odom(x: float, y: float, yaw: float,
                vx: float, vy: float, omega: float) -> bytes:
    """打包雷达里程计帧 (CMD_ODOM)，24 字节载荷。"""
    payload = struct.pack("<6f", x, y, yaw, vx, vy, omega)
    return build_frame(CMD_ODOM, payload)


def encode_depth(class_id: int, center_depth: float,
                 gx: float, gy: float, gz: float,
                 in_range: bool) -> bytes:
    """打包摄像头目标中心深度帧 (CMD_DEPTH)，18 字节载荷。

    center_depth 必须是分割掩膜中心点的深度（稳健中值），而非包围框均值。
    gx/gy/gz 是经手眼标定变换后的夹爪坐标系坐标。
    in_range 供下位机判断目标是否已进入合理夹取范围（到达判断）。
    """
    payload = struct.pack("<B4fB",
                          class_id & 0xFF,
                          center_depth, gx, gy, gz,
                          1 if in_range else 0)
    return build_frame(CMD_DEPTH, payload)


# ==========================================================================
# 下行载荷编码（下位机调用，向上位机发送；此处提供供仿真/联调使用）
# ==========================================================================
def encode_cam_on() -> bytes:
    """打包开摄像头帧 (CMD_CAM_ON)，载荷为空。"""
    return build_frame(CMD_CAM_ON, b"")


def encode_cam_off() -> bytes:
    """打包关摄像头帧 (CMD_CAM_OFF)，载荷为空。"""
    return build_frame(CMD_CAM_OFF, b"")


# ==========================================================================
# 通用增量式帧解析器（上行下行通用，两端均可复用）
# ==========================================================================
@dataclass
class Frame:
    """解析出的一帧，含命令字与载荷。"""
    cmd:     int
    payload: bytes


class FrameParser:
    """增量式帧解析器：喂入任意长度字节流，吐出 CRC 校验通过的完整帧。

    上行（上位机侧）：用于解析下行帧（CMD_CAM_ON / CMD_CAM_OFF）。
    下行（下位机侧）：用于解析上行帧（CMD_ODOM / CMD_DEPTH）。
    上下行帧格式完全一致，两端复用同一个类即可。

    用法：
        parser = FrameParser()
        for frame in parser.feed(raw_bytes):
            handle(frame.cmd, frame.payload)
    """

    def __init__(self, max_buffer: int = 4096) -> None:
        self._buf        = bytearray()
        self._max_buffer = max_buffer

    def feed(self, data: bytes) -> List[Frame]:
        """向解析器喂入新字节，返回本次新完成解析的帧列表（可能为空）。"""
        self._buf.extend(data)
        # 防止异常数据无限堆积缓冲区
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

        返回：
            (Frame, consumed)  解析成功；consumed 为消耗的字节数
            (None,  1)         帧头不符或 CRC 错误；滑动 1 字节继续同步
            (None,  0)         数据不足，等待更多字节
        """
        buf = self._buf

        # 至少需要 HEAD0 HEAD1
        if len(buf) < 2:
            return None, 0

        # 找帧头
        if not (buf[0] == HEAD0 and buf[1] == HEAD1):
            return None, 1      # 不是帧头，跳一字节继续找

        # 至少需要 HEAD0 HEAD1 CMD LEN
        if len(buf) < 4:
            return None, 0

        length = buf[3]
        # total = HEAD0(1) + HEAD1(1) + CMD(1) + LEN(1) + PAYLOAD(length) + CRC(2)
        total = 4 + length + 2

        if len(buf) < total:
            return None, 0      # 帧未收全，等待

        cmd     = buf[2]
        payload = bytes(buf[4 : 4 + length])

        # CRC 校验（范围：CMD + LEN + PAYLOAD）
        recv_crc = struct.unpack("<H", bytes(buf[4 + length : total]))[0]
        calc_crc = crc16_modbus(bytes(buf[2 : 4 + length]))
        if recv_crc != calc_crc:
            return None, 1      # 校验失败，跳一字节重新同步

        return Frame(cmd=cmd, payload=payload), total