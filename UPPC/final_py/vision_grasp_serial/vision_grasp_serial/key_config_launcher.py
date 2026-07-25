""" key_config_launcher.py — HTTP服务 + ROS2 keystroke分发
用法:
  python key_config_launcher.py                  → 同时打开两个窗口
  python key_config_launcher.py --editor-only     → 仅键位编辑器
  python key_config_launcher.py --control-only    → 仅键盘操控
"""
import os, json, webbrowser, threading, time, sys  # os=路径/退出, json=读写配置, webbrowser=打开浏览器, threading=多线程, time=心跳计时, sys=命令行参数
from http.server import HTTPServer, SimpleHTTPRequestHandler  # Python 自带的 HTTP 服务器，不需要装任何第三方库
                                                              # SimpleHTTPRequestHandler = 自动 serve 静态文件（HTML/JS/CSS）

# 和 teleop.py 一样——keymap.json 就在这个文件同目录下
KEYMAP_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "keymap.json")
PORT = 17890                           # HTTP 服务端口号（随便选的，只要不跟别的服务冲突就行）
last_ping = time.time()                # 记录"最后一次收到 HTTP 请求的时间"（= 心跳检测基准）
                                       # time.time() 返回 1970-01-01 到现在走过的秒数（和 C 的 HAL_GetTick 类似但单位是秒）

# ── 按键→动作 映射表（和 teleop.py 完全一样，因为两边都是从 keymap.json 查表）──
WEAPON_IDS = {"sucker1":1,"sucker2":2,"sucker3":3,"sucker4":4,"clamp":6,"servo":7}
# KFS_IDS 的值是 (设备号, 方向) 的元组，和 C 的结构体传两个字段一样
KFS_IDS = {"three_kfs_dec":(1,0),"three_kfs_inc":(1,1),"kfs_spin_dec":(2,0),"kfs_spin_inc":(2,1),
           "main_lift_dec":(3,0),"main_lift_inc":(3,1),"flex_dec":(4,0),"flex_inc":(4,1),"flex_mode":(5,0)}
FLOW_IDS = {"get_kfs":1,"put_kfs":2,"upstairs":3,"downstairs":4,"upslope":6,"up_r1":5,"camera_dbg":7}
ZONE_IDS = {"zone1":1,"zone2":2,"zone3":3,"zone3_prep":4}


class Handler(SimpleHTTPRequestHandler):
    """
    HTTP 请求处理器（类比 C 里面解析串口帧的 dispatch_frame 函数）
    继承 SimpleHTTPRequestHandler = 自带"把文件路径映射到本地文件"的能力
    我们只 override（= 覆盖/重写）三个方法：do_POST / do_GET / _json
    """
    def do_POST(self):
        """处理 POST 请求（浏览器用 POST 来"提交数据"）"""
        global last_ping                # 声明要修改模块级的 last_ping 变量（Python 里要加 global 关键字才能修改外层变量）

        # ── /save：保存键位配置 ──
        if self.path == "/save":        # self.path = URL 路径，如 /save 或 /keystroke
            # 读 HTTP Body 的长度（Content-Length 头）
            length = int(self.headers.get("Content-Length", 0))  # .get(key, default) = 字典取 key，没有就返回 0
            data = json.loads(self.rfile.read(length))            # 读 body → 解析 JSON → Python dict
            with open(KEYMAP_PATH, "w", encoding="utf-8") as f:   # 打开文件准备写入（"w" = 覆盖模式）
                json.dump(data, f, indent=2, ensure_ascii=False)  # 把 dict 写回 JSON 文件，indent=2 = 缩进 2 格美化
            self._json({"ok": True})     # 返回 JSON 响应告诉浏览器"保存成功"
            return

        # ── /keystroke：接收键盘事件 → 发布 ROS2 话题 ──
        if self.path == "/keystroke":
            length = int(self.headers.get("Content-Length", 0))
            data = json.loads(self.rfile.read(length))    # 读 body → 解析成 dict
            # data = {"key":"w", "mode":"single", "action":"down"}
            # .get(key, default) = 安全取字典值，key 不存在时用 default 值（防止崩溃）
            _publish_keystroke(data.get("key",""), data.get("mode","single"), data.get("action",""))
            self._json({"ok": True})
            return

        # 其他 POST 路径 → 404
        self.send_error(404)

    def do_GET(self):
        """处理 GET 请求（浏览器用 GET 来"读数据"）"""
        global last_ping
        last_ping = time.time()          # ★ 任何 GET 请求都刷新心跳（只要有浏览器开着轮询，服务器就不会自毁）

        # ── /ping：心跳检测（浏览器每秒发一次，告诉服务器"我还活着"）──
        if self.path == "/ping":
            self._json({"ok": True})
            return

        # ── /load：返回当前 keymap.json 的内容 ──
        if self.path == "/load":
            with open(KEYMAP_PATH, "r", encoding="utf-8") as f:
                data = json.load(f)      # 读 JSON → Python dict
            self._json(data)             # 把整个 dict 作为 JSON 返回给浏览器
            return

        # ── 根路径 → 重定向到 key_config.html ──
        if self.path in ("/", ""):       # 用户访问 http://localhost:17890 → 自动跳转到 key_config.html
            self.path = "/key_config.html"

        # 其他路径 → 交给父类 SimpleHTTPRequestHandler 处理（它会自动从本地文件系统读取对应文件）
        # 比如 /keyboard_preview.html → 从当前目录读出 keyboard_preview.html 内容 → 返回给浏览器
        super().do_GET()

    def _json(self, data):
        """构造一个 JSON 响应（= 把 Python dict 序列化成 JSON 字符串 → 写 HTTP Response）"""
        self.send_response(200)                           # HTTP 状态码 200 = OK
        self.send_header("Content-Type", "application/json")  # 告诉浏览器：返回的是 JSON
        self.send_header("Access-Control-Allow-Origin", "*")  # CORS 头：允许任何来源的请求（跨域用）
        self.end_headers()                                 # 头写完，准备写 body
        # json.dumps() 把 dict 转成 JSON 字符串，.encode("utf-8") 转成 bytes
        # self.wfile.write() 把 bytes 写到 HTTP 响应流（= 发给浏览器）
        self.wfile.write(json.dumps(data).encode("utf-8"))

    def log_message(self, *args):
        """重写父类的日志方法为空——让 HTTP 服务器安静运行，不往终端打印每次请求的日志"""
        pass  # Python 关键字：什么都不做


# ═══════════════════════════════════════════════════════════ ROS2 发布函数（延迟绑定）

# 这些全局变量一开始都是 None，等 ROS2 初始化成功后才赋值为真正能 publish 的 lambda
# 这样设计的好处：如果没有 ROS2 环境（比如在 Windows 上测试），这些函数调用时什么都不会发生，不会崩溃
_pub_chassis_fn = _pub_weapon_fn = _pub_lift_fn = _pub_kfs_fn = _pub_flow_fn = _pub_zone_fn = _pub_estop_fn = None
# Python 链式赋值：x=y=z=None = 三个变量全赋成 None

def _pub_chassis(vx,vy,vw):
    """发布底盘速度 → /chassis_cmd（Float32MultiArray [vx,vy,vw]）"""
    if _pub_chassis_fn: _pub_chassis_fn(vx,vy,vw)   # 如果 ROS2 初始化成功，就调真正的 publish；否则跳过

def _pub_weapon(dev):
    """发布武器 toggle → /weapon_cmd（Float32 设备号）"""
    if _pub_weapon_fn: _pub_weapon_fn(dev)

def _pub_lift(val):
    """发布抬升速度 → /lift_cmd（Float32 速度值）"""
    if _pub_lift_fn: _pub_lift_fn(val)

def _pub_kfs(dev,dir_val):
    """发布 KFS 档位 → /kfs_pos_cmd（Float32MultiArray [设备号, 方向]）"""
    if _pub_kfs_fn: _pub_kfs_fn(dev,dir_val)

def _pub_flow(val):
    """发布流程函数 → /flow_cmd（Float32 流程号）"""
    if _pub_flow_fn: _pub_flow_fn(val)

def _pub_zone(val):
    """发布业务 zone → /zone_cmd（Float32 zone号）"""
    if _pub_zone_fn: _pub_zone_fn(val)

def _pub_estop(val):
    """发布 PC 急停 → /pc_estop（Float32 1=急停 0=恢复）"""
    if _pub_estop_fn: _pub_estop_fn(val)


def _publish_keystroke(key, mode, action):
    """
    按键→动作 核心分派函数
    输入：key="w", mode="single", action="down"
    输出：查 keymap.json → 找到映射 → 发布对应的 ROS2 话题
    """
    # 每次按键都重新读 keymap.json（文件可能被键位编辑器改过了，保证总是最新）
    try:
        with open(KEYMAP_PATH, "r", encoding="utf-8") as f:
            km = json.load(f)
    except:
        return  # 文件读失败（比如正在被编辑器写入）→ 跳过

    # ── 急停键（Space）立即生效，不走查表 ──
    if key == "space":
        _pub_estop(1.0 if action == "down" else 0.0)
        # Python 的三元表达式：a if 条件 else b  ← 等效 C 的 条件 ? a : b
        if action == "down":
            _pub_chassis(0,0,0)
            _pub_lift(0)
        return

    # ── 内部信号：页面失焦时浏览器发来的清理事件，不经过 keymap 查表 ──
    if key == "__chassis_zero__":
        _pub_chassis(0,0,0); return     # Python 一行可以写多条语句，用分号隔开
    if key == "__lift_zero__":
        _pub_lift(0); return

    # 取出速度参数（.get(key, default) 防止 key 不存在时崩溃）
    sp = km.get("speed", {})

    # ── 底盘键：两种模式（single/auto）都生效 ──
    # km.get("single",{}) = 取 "single" 字段，不存在就返回空字典 {}（防止崩溃）
    # .get("chassis",{})  = 取 "chassis" 子字段，不存在也返回 {}
    ch = (km.get("single",{})).get("chassis", {})
    # .items() = 把字典转成 (key, value) 列表：[("forward","w"), ("backward","s"), ...]
    for act, k in ch.items():            # act="forward", k="w"
        if k == key:                     # 找到了！按键 "w" 匹配到动作 "forward"
            if action == "down":
                fwd = sp.get("chassis_forward", 0.3)    # 取速度值，没有就用默认 0.3
                strf = sp.get("chassis_strafe", 0.3)
                rot = sp.get("chassis_rotate", 30.0)
                if act == "forward":   _pub_chassis(fwd, 0, 0)
                elif act == "backward": _pub_chassis(-fwd, 0, 0)
                elif act == "strafe_l": _pub_chassis(0, -strf, 0)
                elif act == "strafe_r": _pub_chassis(0, strf, 0)
                elif act == "rotate_l": _pub_chassis(0, 0, rot)
                elif act == "rotate_r": _pub_chassis(0, 0, -rot)
            else:                       # action == "up"
                _pub_chassis(0, 0, 0)   # 松手 = 停车
            return  # 找到了就退出，不再往下匹配武器/抬升/KFS

    # ── 单一模式：武器/抬升/KFS ──
    if mode == "single":
        # 武器 toggle：遍历 weapon 映射，找到匹配的按键 → 发设备号
        wp = (km.get("single",{})).get("weapon", {})
        for act, k in wp.items():
            if k == key and action == "down" and act in WEAPON_IDS:
                _pub_weapon(WEAPON_IDS[act])
                return

        # 抬升：按住动松开停
        lf = (km.get("single",{})).get("lift", {})
        for act, k in lf.items():
            if k == key:
                v = sp.get("lift", 2.0)
                if action == "down":
                    if act == "up":        _pub_lift(v)
                    elif act == "down":     _pub_lift(-v)
                    elif act == "up_fast":  _pub_lift(v*2)
                    elif act == "down_fast":_pub_lift(-v*2)
                else:
                    _pub_lift(0)
                return

        # KFS 档位：按一次发一档（设备号, 方向）
        ks = (km.get("single",{})).get("kfs", {})
        for act, k in ks.items():
            if k == key and action == "down" and act in KFS_IDS:
                dev, dir_val = KFS_IDS[act]  # Python 元组解包：dev,dir = (1,0) → dev=1, dir=0
                _pub_kfs(dev, dir_val)
                return

    else:  # mode == "auto"
        # 全自动：流程函数
        fl = (km.get("auto",{})).get("flow", {})
        for act, k in fl.items():
            if k == key and action == "down" and act in FLOW_IDS:
                _pub_flow(FLOW_IDS[act])
                return

        # 全自动：业务 zone
        zn = (km.get("auto",{})).get("zone", {})
        for act, k in zn.items():
            if k == key and action == "down" and act in ZONE_IDS:
                _pub_zone(ZONE_IDS[act])
                return


class HeartbeatServer(HTTPServer):
    """
    心跳服务器（继承 Python 自带的 HTTPServer）
    重写 service_actions() 方法——这个方法是 HTTPServer.serve_forever() 的主循环里
    每隔 ~0.5 秒自动回调一次的钩子函数。我们在这里检查心跳，超时了就杀进程。
    """
    def service_actions(self):
        # time.time() - last_ping = 距离上一次 HTTP 请求过了多少秒
        if time.time() - last_ping > 3:   # 3 秒没有请求 → 认为所有浏览器窗口都关了
            os._exit(0)                   # 硬杀进程（= C 里直接拉闸，不跑任何清理代码）
                                          # os._exit 不是 sys.exit——它不触发异常、不执行 finally


def start():
    """
    主入口函数
    1. 初始化 ROS2（如果可用的话）
    2. 启动 HTTP 服务器
    3. 打开浏览器/pywebview 窗口
    4. 进入心跳检测主循环
    """
    # global 声明：在函数内部修改模块级变量必须先声明
    global _pub_chassis_fn, _pub_weapon_fn, _pub_lift_fn, _pub_kfs_fn
    global _pub_flow_fn, _pub_zone_fn, _pub_estop_fn

    # 切换工作目录到脚本所在目录（保证 HTTP 服务器能找到 key_config.html 等文件）
    # os.path.dirname(os.path.abspath(__file__)) = 这个 .py 文件所在的目录
    os.chdir(os.path.dirname(os.path.abspath(__file__)))

    # 解析命令行参数：sys.argv = ["key_config_launcher.py", "--editor-only"]
    # "--editor-only" in sys.argv → 如果存在这个参数，editor_only = True
    editor_only = "--editor-only" in sys.argv
    control_only = "--control-only" in sys.argv

    # ═══ ROS2 初始化（try/except = 如果 ROS2 环境不存在也不会崩溃）═══
    try:
        import rclpy
        from rclpy.node import Node                     # ROS2 节点基类
        from std_msgs.msg import Float32, Float32MultiArray  # 标准消息类型
        rclpy.init()                                    # 初始化 ROS2
        n = Node("key_config_launcher")                  # 创建一个 ROS2 节点，名字叫 "key_config_launcher"

        # 创建 7 个发布者（= 声明 7 个话题）
        ch = n.create_publisher(Float32MultiArray, "/chassis_cmd", 10)  # 底盘 [Vx,Vy,Vw]
        wp = n.create_publisher(Float32, "/weapon_cmd", 10)             # 武器 设备号
        lf = n.create_publisher(Float32, "/lift_cmd", 10)               # 抬升 速度
        ks = n.create_publisher(Float32MultiArray, "/kfs_pos_cmd", 10)  # KFS [设备号,方向]
        fl = n.create_publisher(Float32, "/flow_cmd", 10)               # 流程号
        zn = n.create_publisher(Float32, "/zone_cmd", 10)               # zone号
        es = n.create_publisher(Float32, "/pc_estop", 10)               # 急停

        # ── 延迟绑定：把全局函数指针指向真正的 ROS2 publish 操作 ──
        # lambda 是 Python 的匿名函数（= C 里写一个单行函数，不需要起名字）
        # lambda vx,vy,vw: ch.publish(Float32MultiArray(data=[float(vx),float(vy),float(vw)]))
        #   等价于 C: void f(float vx,float vy,float vw) { Float32MultiArray msg; msg.data[0]=vx; ...; ch->publish(msg); }
        _pub_chassis_fn = lambda vx,vy,vw: ch.publish(Float32MultiArray(data=[float(vx),float(vy),float(vw)]))
        _pub_weapon_fn  = lambda d: wp.publish(Float32(data=float(d)))
        _pub_lift_fn    = lambda v: lf.publish(Float32(data=float(v)))
        _pub_kfs_fn     = lambda d,dr: ks.publish(Float32MultiArray(data=[float(d),float(dr)]))
        _pub_flow_fn    = lambda v: fl.publish(Float32(data=float(v)))
        _pub_zone_fn    = lambda v: zn.publish(Float32(data=float(v)))
        _pub_estop_fn   = lambda v: es.publish(Float32(data=float(v)))

        # ROS2 需要自己的 spin 循环来驱动回调。但我们这个节点没有订阅（只有发布），
        # 所以只需要一个轻量级的 spin_once 循环保持 ROS2 存活就行
        def ros_spin():
            while rclpy.ok():                                 # rclpy.ok() = ROS2 还没关机
                rclpy.spin_once(n, timeout_sec=0.05)          # 每次 spin 最多 50ms（给其他线程让路）
        # daemon=True = 守护线程：主线程退出后自动被杀，不用手动 join
        threading.Thread(target=ros_spin, daemon=True).start()
        n.get_logger().info("ROS2 connected")
    except Exception as e:
        # ROS2 环境不存在（比如在 Windows 上测试，没装 ROS2）→ 离线模式
        print(f"ROS2 offline: {e}")

    # ═══ HTTP 服务器 ═══
    # HeartbeatServer = 带心跳检测的 HTTPServer（继承 HTTPServer）
    # ("127.0.0.1", 17890) = 只监听本机，端口 17890（外部访问不到，安全）
    server = HeartbeatServer(("127.0.0.1", PORT), Handler)
    # daemon=True：主线程退出后这个线程自动死，不用手动关
    threading.Thread(target=server.serve_forever, daemon=True).start()
    # serve_forever = 死循环监听 HTTP 请求，有请求就创建一个 Handler 线程处理

    url = f"http://127.0.0.1:{PORT}"    # Python f-string：f"http://localhost:{PORT}" → "http://localhost:17890"

    # ═══ 打开用户界面（二选一：pywebview 原生窗口 / 浏览器） ═══
    try:
        import webview          # 尝试加载 pywebview（原生窗口，无地址栏）
        has_webview = True      # 加载成功
    except Exception:
        has_webview = False     # 加载失败 → 降级用浏览器

    if has_webview:
        # pywebview：先 create_window 注册所有窗口，最后一次性 start 打开
        # create_window 不阻塞，可以连续调多个 → start() 一次打开全部窗口
        if not control_only:
            webview.create_window("Key Config", f"{url}/key_config.html",
                                  width=620, height=700, resizable=True, min_size=(480,500))
        if not editor_only:
            webview.create_window("Key Control", f"{url}/keyboard_preview.html",
                                  width=1000, height=520, resizable=True)
        webview.start()          # ★ 这一行阻塞主线程，直到所有窗口关闭
    else:
        # 降级：用操作系统默认浏览器打开（webbrowser.open = 相当于你手动双击 HTML 文件）
        if not control_only:
            webbrowser.open(f"{url}/key_config.html")
        if not editor_only:
            webbrowser.open(f"{url}/keyboard_preview.html")

    # ═══ 主线程保活（心跳检测） ═══
    # 浏览器/HTML 的轮询停了 → last_ping 不再更新 → 3 秒后自动退出进程
    try:
        while True:                         # 死循环（= C 的 for(;;)）
            time.sleep(1)                   # 睡 1 秒（不占 CPU）
            if time.time() - last_ping > 3: # 3 秒没收到请求
                os._exit(0)                 # 硬杀进程
    except KeyboardInterrupt:
        pass  # Ctrl+C → 退出（什么都不做，自然退出）


# Python 入口约定：直接运行 → 调 start()；被 import → 不调用
if __name__ == "__main__":
    start()
