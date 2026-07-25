#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
teleop.py — PC 键盘遥控脚本
用法: python3 teleop.py

启动读取 keymap.json，键盘 → ROS2 话题 → serial_bridge → 串口 → STM32
Tab 切换单一/全自动，Space 急停，ESC 退出

按键行为：
  底盘/抬升 — 按住动松开停（publish 非零/零）
  武器 — 按一次 toggle（publish 设备号）
  KFS — 按一次换一档（publish 设备号+方向）
  全自动 — 按一次触发
"""

import json                # 读 keymap.json，json.load() 把 JSON 文本转成 Python 字典
import os                  # os.path.join() 拼路径，跨平台兼容
import struct              # 序列化：struct.pack('<f', 3.14) 把 float 转成 4 字节
import rclpy               # Python 版 ROS2 客户端库 (= FreeRTOS 调度器)
from rclpy.node import Node  # 所有 ROS2 节点的爹，类比：C 里的 baseModule
from std_msgs.msg import Float32, Float32MultiArray  # 标准消息类型：单个 float / float数组
from pynput import keyboard  # 监听键盘事件的库，拿到 keydown 和 keyup

# ── 常量 ──
# 这个文件在哪，keymap.json 就在哪（同目录）
# os.path.abspath(__file__) = 拿到当前文件的绝对路径
# os.path.dirname(...)       = 拿路径的目录部分（去掉文件名）
# os.path.join(a,b)          = 拼接路径 a\b，Windows 用 \，Linux 用 /
KEYMAP_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "keymap.json")

# 三种操作模式（枚举值，用大写常量表示）
MODE_SINGLE = 0  # 单一机构验证模式（底盘/武器/抬升/KFS 逐个试）
MODE_AUTO   = 1  # 全自动模式（取KFS流程 / zone启动）
MODE_ESTOP  = 2  # 急停模式（所有输出清零，按键无效）

# 武器设备号映射
# Python dict = C 数组 {key: value}，但 key 不限于整数，可以是字符串
# 和 STM32 cmd_dispatch.c 的 0x36 case 对应：PC 发设备号 → STM32 调 sucker1_use() 等
WEAPON_IDS = {
    "sucker1": 1, "sucker2": 2, "sucker3": 3, "sucker4": 4,  # 吸盘 1~4
    "clamp": 6, "servo": 7                                      # 夹爪 / 舵机
}

# KFS 设备号映射
# 和 STM32 cmd_dispatch.c 的 0x37~0x3A case 对应：dev=1→three_kfs, dev=2→kfs_spin ...
KFS_IDS = {
    "three_kfs": 1,  # 三档旋转 KFS
    "kfs_spin": 2,   # 前臂旋转
    "main_lift": 3,  # 主轴抬升
    "flex": 4,        # 伸缩
    "flex_mode": 5    # 伸缩模式切换
}


class Teleop(Node):                        # 继承 rclpy.node.Node = 定义一个 ROS2 节点 (= FreeRTOS 任务)
    def __init__(self):                   # __init__ = C 结构体的初始化函数，创建对象时自动调
        super().__init__("teleop")        # 调用父类 Node 的 __init__，注册节点名叫 "teleop"
                                          # super() = 父类，相当于 C 里 baseModule.Init()

        # 读取 keymap.json → self.km（存成一个 Python dict）
        self._load_keymap()

        # ── 创建发布者（= C 里 CAN_Send 的 "声明"） ──
        # self.xxx 是成员变量，相当于 C 结构体里的字段
        # create_publisher(消息类型, 话题名, 队列长度) = 告诉 ROS2 "我会往这个话题发数据"
        # 队列长度：消峰填谷，话题上有积压时最多缓存几帧
        self.pub_chassis = self.create_publisher(Float32MultiArray, "/chassis_cmd", 10)   # 底盘速度 [Vx,Vy,Vw]
        self.pub_weapon  = self.create_publisher(Float32,          "/weapon_cmd",  10)   # 武器 toggle [设备号]
        self.pub_lift    = self.create_publisher(Float32,          "/lift_cmd",    10)   # 抬升速度 [float]
        self.pub_kfs_pos = self.create_publisher(Float32MultiArray, "/kfs_pos_cmd", 10)  # KFS 档位 [设备号,方向]
        self.pub_flow    = self.create_publisher(Float32,          "/flow_cmd",    10)   # 流程函数 [流程号]
        self.pub_zone    = self.create_publisher(Float32,          "/zone_cmd",    10)   # 业务 zone [zone号]
        self.pub_estop   = self.create_publisher(Float32,          "/pc_estop",    10)   # PC 急停 [1/0]

        # 默认启动 = 单一机构验证模式
        self.mode = MODE_SINGLE
        self._print_mode()               # 打一行日志告知当前模式

        # set() = 空集合（Python 内置数据结构，和 C 里用 uint8_t 数组做标记一样）
        # 用来记录"哪些键正被按住"，实现按键 repeat 去重
        self._held = set()

        # 抬升防抖：记录上一次抬升指令的键，防止松手时误停别的抬升动作
        self._lift_key = None            # None = Python 的 NULL

    def _load_keymap(self):
        """读 keymap.json 到 self.km 字典，并取出快捷键绑定"""
        # with open(...) as f: = C 的 fopen + fclose 自动管理，退出 with 块自动关文件
        with open(KEYMAP_PATH, "r", encoding="utf-8") as f:
            self.km = json.load(f)       # json.load() = 把 JSON 文本解析成 Python dict
                                          # dict["single"]["chassis"]["forward"] → "w"
        # 取出模式切换键、急停键、速度参数（因为频繁使用，缓存在成员变量比每次都 dict 查快）
        self.key_mode_switch = self.km["mode_switch"]       # 默认 "tab"
        self.key_estop       = self.km["estop_toggle"]      # 默认 "space"
        self.spd_ch_fwd  = self.km["speed"]["chassis_forward"]  # 底盘前进速度，默认 0.3
        self.spd_ch_str  = self.km["speed"]["chassis_strafe"]   # 底盘平移速度
        self.spd_ch_rot  = self.km["speed"]["chassis_rotate"]   # 底盘旋转速度
        self.spd_lift    = self.km["speed"]["lift"]             # 抬升速度

    def _print_mode(self):
        """打一行日志，告知当前处于什么模式"""
        # Python dict 也可以用 C 枚举值做 key（和 C 的数组用宏做下标一样）
        msgs = {MODE_SINGLE: ">>> 单一机构验证模式", MODE_AUTO: ">>> 全自动模式",
                MODE_ESTOP: "!!! 急停 !!!"}
        # self.get_logger() = ROS2 节点的日志工具，info() 打 info 级别的日志
        self.get_logger().info(msgs[self.mode])

    # ═══════════════════════════════════════════════════════════ 键盘事件

    def _key_char(self, key):
        """
        把 pynput 的 key 对象转成标准字符串（和 keymap.json 里存的键名格式一致）
        pynput 的 key 对象分两类：
          - 普通字符键：key.char = 'a', '1', ',' 等
          - 特殊键：key.char = None（如 Tab、Space、方向键），需要从 key 的字符串表示里提取
        """
        try:
            # 普通字符键：直接返回 .char 转小写。如果 key.char 是 None → or "" 返回空串
            return (key.char or "").lower()
        except AttributeError:
            # 特殊键：str(key) = "Key.tab" → 去掉 "Key." 前缀 → "tab"
            s = str(key).replace("Key.", "")
            # Python dict.get(s, default) = 如果字典里有 s 就返回对应值，没有就返回 default
            # 这里把 pynput 的特殊键名映射到我们 keymap.json 里用的标准键名
            return {"space": "space", "tab": "tab", "esc": "esc", "enter": "enter",
                    "comma": ",", "period": ".", "up": "up", "down": "down",
                    "left": "left", "right": "right"}.get(s, s.lower())

    def on_press(self, key):
        """
        pynput 回调：键盘按下时触发
        参数 key 是 pynput 的 Key 对象
        """
        kc = self._key_char(key)         # 把 pynput 的 key 对象转成标准字符串
        if not kc:                       # 如果转换结果是空字符串（识别不了的键）→ 忽略
            return

        # ESC = 退出程序
        if kc == "esc":
            self._stop_all()             # 所有执行器清零
            self.get_logger().info("退出")
            return False                 # pynput 约定：返回 False 表示停止 Listener (= 退出程序)

        # ── 按键 repeat 去重 ──
        # 按住 W 不放时，操作系统会重复产生 keydown 事件。用 self._held 集合记录"哪些键正在被按住"。
        # 如果这个键之前已经在 _held 里 → 说明是 repeat 事件 → 跳过，不发 ROS2 指令。
        if self._held:
            self._held.add(kc)           # Python set.add() = 往集合里加一个元素，和 C 的置位 flag 一样
            return

        self._held.add(kc)               # 第一次按下的键：加入 _held，标记"已按下"

        # ── 模式切换 ──
        if kc == self.key_mode_switch:   # 按了 Tab 键（默认）
            if self.mode == MODE_SINGLE:   self.mode = MODE_AUTO   # 单一 → 全自动
            elif self.mode == MODE_AUTO:   self.mode = MODE_SINGLE  # 全自动 → 单一
            self._stop_all()             # 切换模式时清零所有输出，防止之前按住的车轮继续转
            self._print_mode()           # 打日志
            return

        # ── 急停 ──
        if kc == self.key_estop:         # 按了 Space 键（默认）
            if self.mode == MODE_ESTOP:   # 已经在急停 → 退出急停
                self.mode = MODE_SINGLE   # 回到单一模式
                self.pub_estop.publish(Float32(data=0.0))   # 告诉 STM32：急停解除
            else:                         # 不在急停 → 进入急停
                self.mode = MODE_ESTOP    # 急停模式
                self.pub_estop.publish(Float32(data=1.0))   # 告诉 STM32：急停
                self._stop_all()          # 本地也清零
            self._print_mode()
            return

        # 急停模式下：拦截所有按键，不发任何 ROS2 指令
        if self.mode == MODE_ESTOP:
            return

        # 根据当前模式派发按键（单一 / 全自动）
        if self.mode == MODE_SINGLE:
            self._press_single(kc)
        else:
            self._press_auto(kc)

    def on_release(self, key):
        """
        pynput 回调：键盘松开时触发
        和 on_press 对称——按住动/松开停的逻辑在这里实现
        """
        kc = self._key_char(key)         # 和 on_press 一样的转换
        self._held.discard(kc)           # Python set.discard() = 从集合里移除元素（不存在也不报错）

        # 只在单一模式下处理 release（全自动模式的按键都只触发一次，不用 release）
        if self.mode == MODE_SINGLE:
            self._release_single(kc)

    # ═══════════════════════════════════════════════════════════ 单一机构验证模式

    def _press_single(self, kc):
        """单一模式按键：底盘/武器/抬升/KFS 各自分发"""
        # 从 keymap 里取出各个子映射表（Python 链式取 nested dict 的值）
        ch = self.km["single"]["chassis"]  # ch = {"forward":"w","backward":"s",...}
        wp = self.km["single"]["weapon"]   # wp = {"sucker1":"1",...}
        lf = self.km["single"]["lift"]     # lf = {"up":"u","down":"i",...}
        ks = self.km["single"]["kfs"]      # ks = {"three_kfs_dec":"," ,...}

        # ── 底盘：按住的键 → pub_chassis(非零速度) ──
        if   kc == ch["forward"]:   self._pub_chassis( self.spd_ch_fwd, 0.0, 0.0)    # W=前进 Vx=0.3
        elif kc == ch["backward"]:  self._pub_chassis(-self.spd_ch_fwd, 0.0, 0.0)    # S=后退 Vx=-0.3
        elif kc == ch["strafe_l"]:  self._pub_chassis(0.0, -self.spd_ch_str, 0.0)    # A=左移 Vy=-0.3
        elif kc == ch["strafe_r"]:  self._pub_chassis(0.0,  self.spd_ch_str, 0.0)    # D=右移 Vy=0.3
        elif kc == ch["rotate_l"]:  self._pub_chassis(0.0, 0.0,  self.spd_ch_rot)    # Q/arrowLeft=左转
        elif kc == ch["rotate_r"]:  self._pub_chassis(0.0, 0.0, -self.spd_ch_rot)    # E/arrowRight=右转

        # ── 武器：按一次 toggle → pub_weapon(设备号) ──
        # wp.values() = {"1","2","3","4","c","v"}，判断 kc 是否在武器映射里
        elif kc in wp.values():
            # wp.items() 返回 (key, value) 对，遍历找到匹配的
            for name, k in wp.items():
                if kc == k:              # name = "sucker1", k = "1"
                    # WEAPON_IDS["sucker1"] = 1，把这个设备号发给 STM32
                    self.pub_weapon.publish(Float32(data=float(WEAPON_IDS[name])))
                    # Python 的 f-string（f"xxx{变量}"）：把变量值嵌入字符串
                    self.get_logger().info(f"  toggle {name}")
                    return

        # ── 抬升：按住动 ──
        elif kc == lf["up"]:             # U=上升
            self._lift_key = "up"
            self.pub_lift.publish(Float32(data= self.spd_lift))      # 正速度 = 上升
        elif kc == lf["down"]:           # I=下降
            self._lift_key = "down"
            self.pub_lift.publish(Float32(data=-self.spd_lift))      # 负速度 = 下降
        elif kc == lf["up_fast"]:        # O=快升
            self._lift_key = "up_fast"
            self.pub_lift.publish(Float32(data= self.spd_lift * 2))  # 速度加倍
        elif kc == lf["down_fast"]:      # P=快降
            self._lift_key = "down_fast"
            self.pub_lift.publish(Float32(data=-self.spd_lift * 2))

        # ── KFS：按一次换一档（方向 0=减，1=加）──
        elif kc == ks["three_kfs_dec"]: self._pub_kfs(KFS_IDS["three_kfs"], 0)    # three_kfs 减一档
        elif kc == ks["three_kfs_inc"]: self._pub_kfs(KFS_IDS["three_kfs"], 1)    # three_kfs 加一档
        elif kc == ks["kfs_spin_dec"]:  self._pub_kfs(KFS_IDS["kfs_spin"],  0)   # kfs_spin 减
        elif kc == ks["kfs_spin_inc"]:  self._pub_kfs(KFS_IDS["kfs_spin"],  1)   # kfs_spin 加
        elif kc == ks["main_lift_dec"]: self._pub_kfs(KFS_IDS["main_lift"], 0)   # main_lift 降
        elif kc == ks["main_lift_inc"]: self._pub_kfs(KFS_IDS["main_lift"], 1)   # main_lift 升
        elif kc == ks["flex_dec"]:      self._pub_kfs(KFS_IDS["flex"],      0)   # 伸缩减
        elif kc == ks["flex_inc"]:      self._pub_kfs(KFS_IDS["flex"],      1)   # 伸缩加
        elif kc == ks["flex_mode"]:     self._pub_kfs(KFS_IDS["flex_mode"], 0)   # 伸缩模式切换

    def _release_single(self, kc):
        """单一模式松键：底盘/抬升松手 → 发零速停车"""
        ch = self.km["single"]["chassis"]
        lf = self.km["single"]["lift"]

        # 底盘键松手 → 发 [0,0,0] 停车
        # Python 的 in 操作符：判断 kc 是否在这个元组 (tuple) 里
        # (a,b,c) = 只读的、不可变的序列，和 C 的 const 数组一样
        if kc in (ch["forward"], ch["backward"], ch["strafe_l"],
                   ch["strafe_r"], ch["rotate_l"], ch["rotate_r"]):
            self._pub_chassis(0.0, 0.0, 0.0)

        # 抬升键松手 → 发 0.0 停止
        if kc in (lf["up"], lf["down"], lf["up_fast"], lf["down_fast"]):
            if kc == lf["up"] or kc == lf["down"] or kc == lf["up_fast"] or kc == lf["down_fast"]:
                self._lift_key = None            # 清空抬升标记
                self.pub_lift.publish(Float32(data=0.0))  # 发零速停车

    # ═══════════════════════════════════════════════════════════ 全自动模式

    def _press_auto(self, kc):
        """全自动模式按键：触发流程函数 / 业务函数"""
        fl = self.km["auto"]["flow"]     # fl = {"get_kfs":"z","put_kfs":"x",...}
        zn = self.km["auto"]["zone"]     # zn = {"zone1":"1","zone2":"2",...}

        # ── 流程函数：按一次触发一个 Process_XXX ──
        if   kc == fl["get_kfs"]:    self.pub_flow.publish(Float32(data=1.0))   # Z = Process_GetKFS
        elif kc == fl["put_kfs"]:    self.pub_flow.publish(Float32(data=2.0))   # X = Process_PutKFS
        elif kc == fl["upstairs"]:   self.pub_flow.publish(Float32(data=3.0))   # C = Process_UpStairs
        elif kc == fl["downstairs"]: self.pub_flow.publish(Float32(data=4.0))   # V = Process_DownStairs
        elif kc == fl["upslope"]:    self.pub_flow.publish(Float32(data=6.0))   # B = Process_UpSlope
        elif kc == fl["up_r1"]:      self.pub_flow.publish(Float32(data=5.0))   # N = Process_UpR1
        elif kc == fl["camera_dbg"]: self.pub_flow.publish(Float32(data=7.0))   # M = 摄像头调试模式

        # ── 业务函数：按一次启动一个 AppZone ──
        elif kc == zn["zone1"]:      self.pub_zone.publish(Float32(data=1.0))   # 1 = AppZone1_Start
        elif kc == zn["zone2"]:      self.pub_zone.publish(Float32(data=2.0))   # 2 = app_zone2_mission_apply
        elif kc == zn["zone3"]:      self.pub_zone.publish(Float32(data=3.0))   # 3 = AppZone3_Start
        elif kc == zn["zone3_prep"]: self.pub_zone.publish(Float32(data=4.0))   # 4 = AppZone3Prep_Start

    # ═══════════════════════════════════════════════════════════ 辅助函数

    def _pub_chassis(self, vx, vy, vw):
        """
        发布底盘速度指令
        Float32MultiArray = float 数组类型，data=[0.3, 0, 0] = 前进 0.3m/s
        float(vx) 是强制类型转换，确保是 Python float（不是 int）
        """
        self.pub_chassis.publish(Float32MultiArray(data=[float(vx), float(vy), float(vw)]))

    def _pub_kfs(self, device_id, direction):
        """
        发布 KFS 档位指令
        [device_id, direction] = [1, 0] → three_kfs 减一档
        device_id: 1=three_kfs 2=kfs_spin 3=main_lift 4=flex 5=flex_mode
        direction: 0=减 1=加
        """
        self.pub_kfs_pos.publish(Float32MultiArray(data=[float(device_id), float(direction)]))

    def _stop_all(self):
        """故障恢复：底盘 + 抬升全发零速"""
        self._pub_chassis(0.0, 0.0, 0.0)
        self.pub_lift.publish(Float32(data=0.0))
        self._lift_key = None

    def run(self):
        """
        启动键盘监听器（= 主循环，等效 C 的 osKernelStart）
        keyboard.Listener(...) 创建一个后台线程监听键盘
        on_press=self.on_press = 按键按下时调我的 on_press 方法
        on_release=self.on_release = 按键松开时调我的 on_release 方法
        with ... as lis: = lis 对象在 with 块结束时自动关闭
        lis.join() = 阻塞当前线程，直到 Listener 停止（返回 False 或异常时停止）
        """
        self.get_logger().info("键盘遥控启动 — Tab切换模式 Space急停 ESC退出")
        self.get_logger().info("底盘/抬升=按住动松开停 | 武器/KFS=按一次toggle | 全自动=按一次触发")
        with keyboard.Listener(on_press=self.on_press, on_release=self.on_release) as lis:
            lis.join()


def main():
    """
    程序入口（等效 C 的 main() 函数）
    rclpy.init()         = HAL_Init + FreeRTOS 初始化
    Teleop()             = 创建 teleop 节点对象（构造函数里注册了 publisher）
    node.run()           = 启动键盘监听器，阻塞等待
    KeyboardInterrupt    = Ctrl+C 触发的中断信号
    finally              = 无论异常如何，都会执行的清理代码块
    rclpy.shutdown()     = 释放 ROS2 资源
    """
    rclpy.init()
    node = Teleop()
    try:
        node.run()
    except KeyboardInterrupt:
        pass             # Ctrl+C → 不做任何事，直接跳到 finally 清理
    finally:
        node.destroy_node()   # 销毁节点
        rclpy.shutdown()      # 关闭 ROS2


# Python 的入口约定：
# 当这个文件被 python teleop.py 直接运行时，__name__ == "__main__" = True
# 当这个文件被 import teleop 导入时，__name__ == "teleop"（不等于 "__main__"）
# 所以：直接运行 → 调 main()；被导入 → 不调 main()
if __name__ == "__main__":
    main()
