#!/usr/bin/env python3
"""
menu_input.py
输入 1/2/3/4：
- 发布 /qr_select 切换图片
- 再发布 /qr_fire=True 让控制节点执行一次
"""

import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool


class MenuInput(Node):
    def __init__(self):
        super().__init__('menu_input')

        self.declare_parameter('select_topic', '/qr_select')
        self.declare_parameter('fire_topic', '/qr_fire')

        self.select_topic = str(self.get_parameter('select_topic').value)
        self.fire_topic = str(self.get_parameter('fire_topic').value)

        self.pub_select = self.create_publisher(String, self.select_topic, 10)
        self.pub_fire = self.create_publisher(Bool, self.fire_topic, 10)

        self.map = {'1': 'Front', '2': 'Back', '3': 'Left', '4': 'Right'}

    def run(self):
        print("\n=== QR 控制菜单 ===")
        print("1: Front（前进一次）")
        print("2: Back （后退一次）")
        print("3: Left （左转一次）")
        print("4: Right（右转一次）")
        print("0: 退出")
        print("==================")

        while rclpy.ok():
            try:
                s = input("请输入选项(0-4)：").strip()
            except (EOFError, KeyboardInterrupt):
                break

            if s == '0':
                break

            if s not in self.map:
                print("无效输入，请输入 0/1/2/3/4")
                continue

            # 1) 切换二维码图片
            name = self.map[s]
            m1 = String()
            m1.data = name
            self.pub_select.publish(m1)

            # 小等一下，确保 image_publisher 切换后 qr_decoder 能读到
            time.sleep(0.2)

            # 2) 扳机：允许执行一次
            m2 = Bool()
            m2.data = True
            self.pub_fire.publish(m2)

            print(f"[menu] 已选择 {name}：将执行一次动作")

        print("退出 menu_input")


def main(args=None):
    rclpy.init(args=args)
    node = MenuInput()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

