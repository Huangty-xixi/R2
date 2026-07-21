#!/usr/bin/env python3
"""
qr_select_from_file.py
你输入 1..N：
1) 从 commands.txt 读取对应行文本
2) 生成二维码图片到 ~/qr_ws/dyn_qr/current.png
3) 发布 /qr_select = 'current.png' 让 image_publisher 切换画面
4) 发布 /qr_fire=True 触发“只执行一次”
"""

import os
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import qrcode


class QRSelectFromFile(Node):
    def __init__(self):
        super().__init__('qr_select_from_file')

        self.declare_parameter('commands_file', os.path.expanduser('~/qr_ws/commands.txt'))
        self.declare_parameter('output_dir', os.path.expanduser('~/qr_ws/dyn_qr'))
        self.declare_parameter('output_name', 'current.png')
        self.declare_parameter('select_topic', '/qr_select')
        self.declare_parameter('fire_topic', '/qr_fire')

        self.commands_file = str(self.get_parameter('commands_file').value)
        self.output_dir = str(self.get_parameter('output_dir').value)
        self.output_name = str(self.get_parameter('output_name').value)
        self.select_topic = str(self.get_parameter('select_topic').value)
        self.fire_topic = str(self.get_parameter('fire_topic').value)

        os.makedirs(self.output_dir, exist_ok=True)

        self.pub_select = self.create_publisher(String, self.select_topic, 10)
        self.pub_fire = self.create_publisher(Bool, self.fire_topic, 10)

    def load_commands(self):
        if not os.path.exists(self.commands_file):
            print(f"[ERR] 找不到 {self.commands_file}")
            return []
        lines = []
        with open(self.commands_file, 'r', encoding='utf-8') as f:
            for line in f:
                s = line.strip()
                if not s or s.startswith('#'):
                    continue
                lines.append(s)
        return lines

    def gen_qr(self, text: str) -> str:
        out_path = os.path.join(self.output_dir, self.output_name)
        img = qrcode.make(text)
        img.save(out_path)
        return out_path

    def run(self):
        while rclpy.ok():
            cmds = self.load_commands()
            if not cmds:
                print("commands.txt 没有有效内容，退出")
                return

            print("\n=== commands.txt 菜单（输入数字生成二维码并触发一次）===")
            for i, cmd in enumerate(cmds, start=1):
                print(f"{i}: {cmd}")
            print("r: 重新加载文件")
            print("0: 退出")
            print("================================================")

            s = input("请输入选项：").strip()
            if s == '0':
                break
            if s.lower() == 'r':
                continue
            if not s.isdigit():
                print("请输入数字 / r / 0")
                continue

            idx = int(s)
            if idx < 1 or idx > len(cmds):
                print("超出范围")
                continue

            text = cmds[idx - 1]
            out_path = self.gen_qr(text)
            print(f"[gen] 已生成二维码: {out_path}")
            print(f"[gen] 内容: {text}")

            # 1) 切换图像为 current.png
            msg_sel = String()
            msg_sel.data = os.path.basename(out_path)  # 'current.png'
            self.pub_select.publish(msg_sel)

            # 等一下让图像切换 + 识别稳定
            time.sleep(0.2)

            # 2) 发扳机：允许执行一次
            msg_fire = Bool()
            msg_fire.data = True
            self.pub_fire.publish(msg_fire)

            print("[fire] 已触发一次执行")

        print("退出 qr_select_from_file")


def main(args=None):
    rclpy.init(args=args)
    node = QRSelectFromFile()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

