#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
serial_spy.py  ——  串口透明监听工具
=====================================
通过 socat 建立透明代理，在不中断 serial_bridge_node 的前提下
旁听所有串口字节流，彩色解析每一帧的方向、命令和字段。

使用方法（两步）：

  步骤 1：终端 A 启动透明代理（保持运行）
    python3 serial_spy.py --setup
    # 它会打印需要修改的串口参数，并在后台启动 socat 代理
    # 实际串口 /dev/ttyUSB_serial <-> 虚拟串口 /tmp/ttyUSB_tap
    # serial_bridge 改连 /tmp/ttyUSB_tap，通信完全透明

  步骤 2：终端 B 查看帧流
    python3 serial_spy.py --watch

  一键模式（自动做步骤1+2，适合临时调试）：
    python3 serial_spy.py --auto --real-port /dev/ttyUSB_serial

  独立模式（停掉 serial_bridge 后，直接监听真实串口）：
    python3 serial_spy.py --direct --real-port /dev/ttyUSB_serial
"""

import argparse
import os
import re
import struct
import subprocess
import sys
import time
import threading

# ─── 颜色 ────────────────────────────────────────────────────────────────────
R   = "\033[91m"
G   = "\033[92m"
Y   = "\033[93m"
C   = "\033[96m"
B   = "\033[94m"
DIM = "\033[2m"
N   = "\033[0m"
BD  = "\033[1m"

# ─── 协议 ────────────────────────────────────────────────────────────────────
HEAD0, HEAD1 = 0xA5, 0x5A

DOWN_CMDS = {          # 下位机 → 上位机
    0x10: "CMD_ACK",
    0x12: "CMD_STATUS",
    0x13: "CMD_ZONE_I_INFO",
    0x14: "CMD_DOCK_OK",
    0x15: "CMD_GO_ZONE_I",
}
UP_CMDS = {            # 上位机 → 下位机
    0x01: "CMD_ODOM",
    0x03: "CMD_KFS",
    0x06: "CMD_KFS_LATERAL_ERR",
}
ALL_CMDS = {**DOWN_CMDS, **UP_CMDS}

RC_STATUS = {0:"IDLE", 1:"MOVING", 2:"REACHED", 3:"GRABBING", 4:"DONE"}

PIPE_PATH = "/tmp/serial_spy_pipe"
SOCAT_PID = "/tmp/serial_spy.pid"

# ─── 帧解析 ──────────────────────────────────────────────────────────────────
def calc_chk(cmd, payload):
    n = len(payload)
    chk = cmd ^ (n & 0xFF) ^ ((n >> 8) & 0xFF)
    for b in payload:
        chk ^= b
    return chk & 0xFF

def parse_fields(cmd, payload):
    try:
        if cmd == 0x12:
            st = payload[0] if payload else 0
            return f"  status={st} {Y}({RC_STATUS.get(st,'?')}){N}"
        if cmd == 0x15:
            return f"  {BD}{Y}→ 触发开摄像头！{N}"
        if cmd == 0x14:
            return f"  {BD}{G}dock_ok=True{N}"
        if cmd == 0x10:
            acked = f"0x{payload[0]:02X}" if payload else "?"
            return f"  acked={acked} result={payload[1] if len(payload)>1 else '?'}"
        if cmd == 0x01 and len(payload) >= 24:
            p0, p1, z, roll, pitch, yaw = struct.unpack("<6f", payload[:24])
            return f"  x={p0:.3f} y={p1:.3f} yaw={yaw:.1f}°"
        if cmd == 0x03 and len(payload) >= 14:
            num, cid = payload[0], payload[1]
            cx, cy, cz = struct.unpack("<3f", payload[2:14])
            return f"  num={num} class={cid} xyz=({cx:.3f},{cy:.3f},{cz:.3f})"
        if cmd == 0x06 and len(payload) >= 12:
            cx, cy, cz = struct.unpack("<3f", payload[:12])
            return f"  xyz=({cx:.3f},{cy:.3f},{cz:.3f})"
    except Exception:
        pass
    return ""

def hex_str(data, max_bytes=20):
    h = " ".join(f"{b:02X}" for b in data[:max_bytes])
    return h + ("…" if len(data) > max_bytes else "")

def try_parse_frame(buf):
    """返回 (cmd, payload, raw_bytes, consumed) 或 (None, None, None, 1 丢弃)"""
    if len(buf) < 2:
        return None, None, None, 0
    if not (buf[0] == HEAD0 and buf[1] == HEAD1):
        return None, None, None, 1          # 丢弃 1 字节
    if len(buf) < 5:
        return None, None, None, 0          # 等更多数据
    cmd    = buf[2]
    length = buf[3] | (buf[4] << 8)
    if length > 256:
        return None, None, None, 1
    total = 5 + length + 1
    if len(buf) < total:
        return None, None, None, 0
    payload = bytes(buf[5:5 + length])
    chk     = buf[total - 1]
    if chk != calc_chk(cmd, payload):
        return None, None, None, 1          # 校验失败
    return cmd, payload, bytes(buf[:total]), total

# ─── 显示帧 ──────────────────────────────────────────────────────────────────
_t0 = time.monotonic()
_rx_count = {}
_bad_bytes = [0]

def show_frame(cmd, payload, raw, direction_char):
    """direction_char: '<' 下位机→上位机(接收), '>' 上位机→下位机(发送)"""
    elapsed = time.monotonic() - _t0
    is_rx   = (direction_char == '<')
    color   = R if is_rx else G
    arrow   = f"{color}{'↓ RX'if is_rx else '↑ TX'}{N}"
    name    = ALL_CMDS.get(cmd, f"CMD_0x{cmd:02X}")
    fields  = parse_fields(cmd, payload)
    _rx_count[cmd] = _rx_count.get(cmd, 0) + 1

    ts = f"{DIM}[{elapsed:9.3f}s]{N}"
    print(f"{ts} {arrow} {color}{BD}{name:<22}{N}(0x{cmd:02X}){fields}")
    print(f"           {DIM}[{hex_str(raw)}]  {len(raw)}B{N}")

# ─── 解析器状态 ──────────────────────────────────────────────────────────────
class StreamParser:
    def __init__(self, direction):
        self._buf = bytearray()
        self._dir = direction   # '<' or '>'

    def feed(self, data: bytes):
        self._buf.extend(data)
        while self._buf:
            cmd, payload, raw, consumed = try_parse_frame(self._buf)
            if consumed == 0:
                break
            del self._buf[:consumed]
            if cmd is not None:
                show_frame(cmd, payload, raw, self._dir)
            else:
                _bad_bytes[0] += 1

# ─── 独立模式：直接打开串口 ──────────────────────────────────────────────────
def mode_direct(port, baud):
    """停掉 serial_bridge 后，直接监听真实串口（只能看到下位机发来的帧）"""
    try:
        import serial
    except ImportError:
        print("pip install pyserial"); sys.exit(1)

    print(f"\n{BD}══ 独立监听模式 ══{N}")
    print(f"端口: {C}{port}{N}  波特率: {C}{baud}{N}")
    print(f"{Y}注意：此模式只能看到下位机发来的帧（RX方向），不影响真实通信需停掉 serial_bridge{N}\n")
    print(f"{'时间':>12}  {'方向':<6}  {'命令':<22}  字段")
    print("─" * 70)

    parser = StreamParser('<')
    try:
        ser = serial.Serial(port, baud, timeout=0.02)
    except Exception as e:
        print(f"{R}串口打开失败: {e}{N}"); sys.exit(1)

    t_last_stat = time.monotonic()
    try:
        while True:
            data = ser.read(256)
            if data:
                parser.feed(data)
            # 每 5 秒打印一次"等待中"提示（如果没有数据）
            if time.monotonic() - t_last_stat > 5.0:
                t_last_stat = time.monotonic()
                total = sum(_rx_count.values())
                if total == 0:
                    print(f"{DIM}[{time.monotonic()-_t0:.1f}s] 等待下位机发送帧...（已丢弃噪声字节: {_bad_bytes[0]}）{N}")
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        _print_stats()

# ─── socat 透明代理模式 ──────────────────────────────────────────────────────
def mode_setup(real_port, baud, tap_port="/tmp/ttyUSB_tap"):
    """
    建立 socat 透明代理：
      serial_bridge → tap_port（虚拟）↔ real_port（硬件）
    同时把所有字节流写入 PIPE_PATH 供 --watch 解析。
    """
    if not shutil_which("socat"):
        print(f"{R}未安装 socat：sudo apt install socat{N}"); sys.exit(1)

    # 创建命名管道
    if os.path.exists(PIPE_PATH):
        os.remove(PIPE_PATH)
    os.mkfifo(PIPE_PATH)

    cmd = (
        f"socat -v -x "
        f"PTY,link={tap_port},raw,echo=0,b{baud} "
        f"{real_port},raw,echo=0,b{baud} "
        f"2>{PIPE_PATH}"
    )
    print(f"\n{BD}══ 透明代理已启动 ══{N}")
    print(f"  真实串口 : {C}{real_port}{N}")
    print(f"  虚拟串口 : {C}{tap_port}{N}  ← serial_bridge 改用此端口")
    print(f"  监听管道 : {C}{PIPE_PATH}{N}\n")
    print(f"{Y}请将 serial_bridge 的 port 参数改为 {tap_port}{N}")
    print(f"  ros2 run vision_grasp_serial serial_bridge_node \\")
    print(f"    --ros-args -p port:={tap_port} -p baudrate:={baud}\n")
    print(f"然后在另一个终端运行：  python3 serial_spy.py --watch\n")

    proc = subprocess.Popen(cmd, shell=True)
    with open(SOCAT_PID, "w") as f:
        f.write(str(proc.pid))
    try:
        proc.wait()
    except KeyboardInterrupt:
        proc.terminate()
    finally:
        if os.path.exists(PIPE_PATH):
            os.remove(PIPE_PATH)

def shutil_which(name):
    import shutil
    return shutil.which(name) is not None

def mode_watch():
    """读取 socat -v -x 输出管道，解析并彩色显示帧"""
    print(f"\n{BD}══ 帧流监视 ══{N}  (等待 socat 代理启动...)\n")
    print(f"{'时间':>12}  {'方向':<6}  {'命令':<22}  字段")
    print("─" * 70)

    # socat -v -x 输出格式：
    # > 时间戳 length=N from=0 to=N-1
    #  xx xx xx xx ...
    # < 时间戳 ...

    rx_parser = StreamParser('<')
    tx_parser = StreamParser('>')

    direction  = None
    hex_accum  = bytearray()

    try:
        with open(PIPE_PATH, 'r', errors='replace') as f:
            for line in f:
                line = line.rstrip()
                # 方向行
                m = re.match(r'^([<>])\s+\d{4}', line)
                if m:
                    if direction is not None and hex_accum:
                        # 提交上一帧
                        (rx_parser if direction == '<' else tx_parser).feed(bytes(hex_accum))
                        hex_accum.clear()
                    direction = m.group(1)
                    continue
                # hex 数据行（socat -x 格式：" xx xx xx ..."）
                hex_part = re.findall(r'\b([0-9a-fA-F]{2})\b', line)
                if hex_part and direction is not None:
                    hex_accum.extend(int(h, 16) for h in hex_part)
    except FileNotFoundError:
        print(f"{R}管道不存在，请先运行: python3 serial_spy.py --setup{N}")
    except KeyboardInterrupt:
        pass
    finally:
        _print_stats()

def _print_stats():
    elapsed = time.monotonic() - _t0
    print(f"\n{'─'*70}")
    print(f"{BD}统计（{elapsed:.1f}s）{N}")
    total = sum(_rx_count.values())
    print(f"  解析帧总数: {total}   丢弃字节: {_bad_bytes[0]}")
    for cmd in sorted(_rx_count):
        name = ALL_CMDS.get(cmd, f"0x{cmd:02X}")
        cnt  = _rx_count[cmd]
        print(f"  {name:<26} {cnt:5d} 帧  ({cnt/elapsed:.1f} Hz)")

# ─── 入口 ────────────────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser(description="串口透明监听工具")
    ap.add_argument("--direct", action="store_true",
                    help="独立模式：停掉 serial_bridge 后直接监听真实串口")
    ap.add_argument("--setup",  action="store_true",
                    help="透明代理模式：建立 socat 代理（保持运行）")
    ap.add_argument("--watch",  action="store_true",
                    help="帧流显示：配合 --setup 使用")
    ap.add_argument("--auto",   action="store_true",
                    help="自动模式：一键 setup+watch")
    ap.add_argument("--real-port", default="/dev/ttyUSB_serial")
    ap.add_argument("--tap-port",  default="/tmp/ttyUSB_tap")
    ap.add_argument("--baud",  type=int, default=115200)
    args = ap.parse_args()

    if args.direct:
        mode_direct(args.real_port, args.baud)
    elif args.setup:
        mode_setup(args.real_port, args.baud, args.tap_port)
    elif args.watch:
        mode_watch()
    elif args.auto:
        t = threading.Thread(
            target=mode_setup,
            args=(args.real_port, args.baud, args.tap_port),
            daemon=True)
        t.start()
        time.sleep(1.5)
        mode_watch()
    else:
        ap.print_help()
        print(f"\n{Y}最快开始：{N}")
        print(f"  # 方案A（停掉 serial_bridge）：")
        print(f"  python3 serial_spy.py --direct\n")
        print(f"  # 方案B（不停 serial_bridge）：")
        print(f"  python3 serial_spy.py --setup   # 终端A")
        print(f"  python3 serial_spy.py --watch   # 终端B")

if __name__ == "__main__":
    main()