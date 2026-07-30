#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
raw_dump.py  ——  串口原始字节 hex 转储
只管收字节，不做任何过滤，按时间分组打印，帮助判断实际收到了什么。

用法：
    python3 raw_dump.py                         # 默认 /dev/ttyUSB_serial 115200
    python3 raw_dump.py --port /dev/ttyUSB_serial --baud 115200
    python3 raw_dump.py --baud 9600             # 怀疑波特率不匹配时换着试
"""
import argparse, sys, time

try:
    import serial as pyserial
except ImportError:
    print("pip install pyserial"); sys.exit(1)

COLORS = {
    "a5": "\033[92m",   # 绿  ← A5（帧头第1字节）
    "5a": "\033[96m",   # 青  ← 5A（帧头第2字节）
    "15": "\033[93m",   # 黄  ← CMD_GO_ZONE_I
    "12": "\033[91m",   # 红  ← CMD_STATUS
    "14": "\033[91m",   # 红  ← CMD_DOCK_OK
}
RST = "\033[0m"
DIM = "\033[2m"
BD  = "\033[1m"

def colorize(hexbyte: str) -> str:
    c = COLORS.get(hexbyte.lower(), "")
    return f"{c}{hexbyte}{RST}" if c else hexbyte

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port",  default="/dev/ttyUSB_serial")
    ap.add_argument("--baud",  type=int, default=115200)
    ap.add_argument("--width", type=int, default=16, help="每行字节数")
    args = ap.parse_args()

    print(f"\n{BD}{'='*60}{RST}")
    print(f"{BD}  串口原始字节转储{RST}")
    print(f"  端口: \033[96m{args.port}\033[0m   波特率: \033[96m{args.baud}\033[0m")
    print(f"  高亮: \033[92mA5\033[0m \033[96m5A\033[0m=帧头  \033[93m15\033[0m=GO_ZONE_I  \033[91m12/14\033[0m=STATUS/DOCK")
    print(f"  按 Ctrl+C 退出")
    print(f"{BD}{'='*60}{RST}\n")

    try:
        ser = pyserial.Serial(args.port, args.baud, timeout=0.05)
    except Exception as e:
        print(f"\033[91m串口打开失败: {e}\033[0m"); sys.exit(1)

    total  = 0
    t0     = time.monotonic()
    t_last = t0
    buf    = bytearray()
    row    = 0        # 当前行已有字节数

    def flush_row(force=False):
        nonlocal row
        if force and buf:
            # 补齐剩余行
            pass
        if row > 0:
            # 打印 ASCII 侧
            ascii_part = "".join(
                chr(b) if 32 <= b < 127 else "."
                for b in buf[-row:]
            )
            pad = "   " * (args.width - row)
            print(f"  {DIM}{pad}{ascii_part}{RST}")
            row = 0

    try:
        while True:
            data = ser.read(256)
            if not data:
                # 超时，如果当前行有内容就换行打印
                if row > 0:
                    flush_row(force=True)
                continue

            now = time.monotonic()
            elapsed = now - t0
            gap     = now - t_last
            t_last  = now
            total  += len(data)

            # 时间戳行（间隔 > 50ms 才打，避免刷屏）
            if gap > 0.05 or row == 0:
                if row > 0:
                    flush_row(force=True)
                rate = total / elapsed if elapsed > 0 else 0
                print(f"\n{DIM}[{elapsed:8.3f}s  +{gap*1000:5.1f}ms  "
                      f"共{total}B  {rate:.0f}B/s]{RST}")

            for b in data:
                buf.append(b)
                hx = f"{b:02x}"
                col_hx = colorize(hx)

                if row == 0:
                    print("  ", end="")

                print(f"{col_hx} ", end="", flush=True)
                row += 1

                if row >= args.width:
                    ascii_part = "".join(
                        chr(x) if 32 <= x < 127 else "."
                        for x in buf[-args.width:]
                    )
                    print(f" {DIM}{ascii_part}{RST}")
                    row = 0

    except KeyboardInterrupt:
        if row > 0:
            flush_row(force=True)
    finally:
        ser.close()
        elapsed = time.monotonic() - t0
        print(f"\n{BD}{'─'*60}{RST}")
        print(f"  共接收: {total} 字节   用时: {elapsed:.1f}s   "
              f"平均: {total/elapsed:.0f} B/s")
        print(f"{BD}{'─'*60}{RST}\n")

if __name__ == "__main__":
    main()