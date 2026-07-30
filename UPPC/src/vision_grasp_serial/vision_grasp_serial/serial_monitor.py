#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
serial_monitor.py  ——  串口通讯帧实时监视器
=============================================
连接到串口（与 serial_bridge_node 同一个端口），
实时解析并彩色显示上下位机之间的每一帧通讯内容。

使用方法：
    python3 serial_monitor.py
    python3 serial_monitor.py --port /dev/ttyUSB_serial --baud 115200

注意：serial_bridge_node 已经占用了串口，本脚本无法与其共享同一端口。
      请使用 socat 建立透明监听（见下方 Phase B 使用说明），
      或在 serial_bridge_node 停止后单独运行本脚本测试下位机帧。
"""

import argparse
import struct
import sys
import time
import os

try:
    import serial as pyserial
    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False

# ─── 颜色 ────────────────────────────────────────────────────────────────────
R  = "\033[91m"   # 红（接收帧）
G  = "\033[92m"   # 绿（上行/发送提示）
Y  = "\033[93m"   # 黄（警告）
C  = "\033[96m"   # 青（字段值）
B  = "\033[94m"   # 蓝（INFO）
DIM= "\033[2m"    # 暗
N  = "\033[0m"    # 重置
BOLD="\033[1m"

# ─── 协议定义 ─────────────────────────────────────────────────────────────────
HEAD0, HEAD1 = 0xA5, 0x5A

# 下位机 → 上位机（下行，此脚本能看到的）
DOWN_CMDS = {
    0x10: "CMD_ACK",
    0x12: "CMD_STATUS",
    0x13: "CMD_ZONE_I_INFO",
    0x14: "CMD_DOCK_OK",
    0x15: "CMD_GO_ZONE_I",
    0x20: "CMD_DEBUG_HEADING",
    0x21: "CMD_DEBUG_NAV",
}

# 上位机 → 下位机（上行，此脚本看不到，但可以从 ROS 话题推断）
UP_CMDS = {
    0x01: "CMD_ODOM",
    0x02: "CMD_PATH",
    0x03: "CMD_KFS",
    0x05: "CMD_ZONE_I_PATH",
    0x06: "CMD_KFS_LATERAL_ERR",
}

RC_STATUS = {0: "IDLE", 1: "MOVING", 2: "REACHED", 3: "GRABBING", 4: "DONE"}

def calc_chk(cmd, payload):
    n = len(payload)
    chk = cmd ^ (n & 0xFF) ^ ((n >> 8) & 0xFF)
    for b in payload:
        chk ^= b
    return chk & 0xFF

def hex_str(data):
    return " ".join(f"{b:02X}" for b in data)

def parse_payload(cmd, payload):
    """解析载荷，返回可读字段字符串"""
    try:
        if cmd == 0x12:  # CMD_STATUS
            st = payload[0] if payload else 0
            return f"status={st} ({RC_STATUS.get(st, '?')})"
        elif cmd == 0x14:  # CMD_DOCK_OK
            return "dock_ok=True"
        elif cmd == 0x15:  # CMD_GO_ZONE_I
            return "→ 触发开摄像头"
        elif cmd == 0x10:  # CMD_ACK
            acked = f"0x{payload[0]:02X}" if payload else "?"
            result = payload[1] if len(payload) > 1 else 0
            return f"acked_cmd={acked} result={result}"
        elif cmd == 0x13:  # CMD_ZONE_I_INFO
            num = payload[0] if payload else 0
            parts = []
            for i in range(num):
                base = 1 + i * 2
                if base + 1 < len(payload):
                    parts.append(f"block{payload[base]}/type{payload[base+1]}")
            return f"num={num} detections=[{', '.join(parts)}]"
        elif cmd == 0x01:  # CMD_ODOM（上行）
            if len(payload) >= 24:
                p0, p1, z, roll, pitch, yaw = struct.unpack("<6f", payload[:24])
                return f"x={p0:.3f} y={p1:.3f} yaw={yaw:.1f}°"
        elif cmd == 0x03:  # CMD_KFS（上行）
            if len(payload) >= 14:
                num = payload[0]
                cid = payload[1]
                cx, cy, cz = struct.unpack("<3f", payload[2:14])
                return f"num={num} class={cid} xyz=({cx:.3f},{cy:.3f},{cz:.3f})"
        elif cmd == 0x06:  # CMD_KFS_LATERAL_ERR（上行）
            if len(payload) >= 12:
                cx, cy, cz = struct.unpack("<3f", payload[:12])
                return f"xyz=({cx:.3f},{cy:.3f},{cz:.3f})"
    except Exception:
        pass
    return ""

class FrameMonitor:
    def __init__(self, port, baud):
        self.port  = port
        self.baud  = baud
        self._buf  = bytearray()
        self._total_rx   = 0
        self._total_bad  = 0
        self._cmd_counts = {}
        self._t0 = time.monotonic()

    def run(self):
        print(f"\n{BOLD}{'='*62}{N}")
        print(f"{BOLD}  串口通讯帧实时监视器{N}")
        print(f"  端口: {C}{self.port}{N}  波特率: {C}{self.baud}{N}")
        print(f"{BOLD}{'='*62}{N}")
        print(f"  {R}■ 红色{N} = 下位机 → 上位机（下行帧，本脚本接收）")
        print(f"  {G}■ 绿色{N} = 上位机 → 下位机（上行帧，仅在透传模式下可见）")
        print(f"  按 Ctrl+C 退出并显示统计\n")

        try:
            ser = pyserial.Serial(self.port, self.baud, timeout=0.02)
        except Exception as e:
            print(f"{R}[错误] 串口打开失败: {e}{N}")
            print(f"\n{Y}提示: serial_bridge_node 正在运行时无法同时打开同一串口。")
            print(f"请先停止 serial_bridge_node，或使用 socat 透传模式（见脚本头部说明）。{N}")
            sys.exit(1)

        print(f"{B}[INFO] 串口已打开，等待数据...{N}\n")

        try:
            while True:
                data = ser.read(256)
                if data:
                    self._feed(data)
        except KeyboardInterrupt:
            pass
        finally:
            ser.close()
            self._print_stats()

    def _feed(self, data):
        self._buf.extend(data)
        while True:
            frame, consumed = self._try_parse()
            if consumed == 0:
                break
            del self._buf[:consumed]
            if frame:
                self._print_frame(*frame)

    def _try_parse(self):
        buf = self._buf
        # 跳过非帧头字节
        while len(buf) >= 2 and not (buf[0] == HEAD0 and buf[1] == HEAD1):
            self._total_bad += 1
            del buf[0]
        if len(buf) < 5:
            return None, 0
        cmd    = buf[2]
        length = buf[3] | (buf[4] << 8)
        if length > 128:
            return None, 1
        total = 5 + length + 1
        if len(buf) < total:
            return None, 0
        payload  = bytes(buf[5:5 + length])
        recv_chk = buf[total - 1]
        calc     = calc_chk(cmd, payload)
        if recv_chk != calc:
            self._total_bad += 1
            return None, 1
        return (cmd, payload, bytes(buf[:total])), total

    def _print_frame(self, cmd, payload, raw):
        self._total_rx += 1
        self._cmd_counts[cmd] = self._cmd_counts.get(cmd, 0) + 1
        elapsed = time.monotonic() - self._t0

        is_down = cmd in DOWN_CMDS
        is_up   = cmd in UP_CMDS
        color   = R if is_down else (G if is_up else Y)
        direction = "↓ RX" if is_down else ("↑ TX" if is_up else "?? ")
        cmd_name = DOWN_CMDS.get(cmd, UP_CMDS.get(cmd, f"CMD_0x{cmd:02X}"))
        fields   = parse_payload(cmd, payload)

        ts   = f"{DIM}[{elapsed:8.3f}s]{N}"
        head = f"{color}{BOLD}{direction} {cmd_name:<22}(0x{cmd:02X}){N}"
        raw_s = f"{DIM}raw=[{hex_str(raw[:min(len(raw),20)])}{'...' if len(raw)>20 else ''}]{N}"
        flds  = f"  {C}{fields}{N}" if fields else ""

        print(f"{ts} {head}{flds}")
        print(f"         {raw_s}")

    def _print_stats(self):
        elapsed = time.monotonic() - self._t0
        print(f"\n{BOLD}{'='*62}{N}")
        print(f"{BOLD}  统计（运行 {elapsed:.1f}s）{N}")
        print(f"  总接收帧: {self._total_rx}")
        print(f"  丢弃字节: {self._total_bad}")
        if self._cmd_counts:
            print(f"  各命令计数:")
            for cmd, cnt in sorted(self._cmd_counts.items()):
                name = DOWN_CMDS.get(cmd, UP_CMDS.get(cmd, f"0x{cmd:02X}"))
                print(f"    {name:<26} {cnt:6d} 帧  "
                      f"({cnt/elapsed:.1f} Hz)" if elapsed > 0 else "")
        print(f"{BOLD}{'='*62}{N}")


def main():
    ap = argparse.ArgumentParser(description="串口通讯帧实时监视器")
    ap.add_argument("--port", default="/dev/ttyUSB_serial")
    ap.add_argument("--baud", type=int, default=115200)
    args = ap.parse_args()

    if not HAS_SERIAL:
        print("未安装 pyserial：pip install pyserial")
        sys.exit(1)

    FrameMonitor(args.port, args.baud).run()


if __name__ == "__main__":
    main()