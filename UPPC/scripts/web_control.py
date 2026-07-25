#!/usr/bin/env python3.10
"""
web_control.py — 极简 Web 遥控面板
  http://<NUC_IP>:17890/control.html  → 键盘操控
  http://<NUC_IP>:17890/editor.html   → 键位编辑
"""
import os, json, threading, struct
from http.server import HTTPServer, SimpleHTTPRequestHandler

# ── 键位映射表 ──
WEAPON_IDS = {"sucker1":1,"sucker2":2,"sucker3":3,"sucker4":4,"clamp":6,"servo":7}
KFS_IDS = {"three_kfs_dec":(1,0),"three_kfs_inc":(1,1),"kfs_spin_dec":(2,0),"kfs_spin_inc":(2,1),
           "main_lift_dec":(3,0),"main_lift_inc":(3,1),"flex_dec":(4,0),"flex_inc":(4,1),"flex_mode":(5,0)}
FLOW_IDS = {"get_kfs":1,"put_kfs":2,"upstairs":3,"downstairs":4,"upslope":6,"up_r1":5,"camera_dbg":7}
ZONE_IDS = {"zone1":1,"zone2":2,"zone3":3,"zone3_prep":4}

# ── ROS2 publishers (延迟绑定) ──
_pub = {}

def _publish_keystroke(key, mode, action):
    with open(KEYMAP_PATH, "r", encoding="utf-8") as f:
        km = json.load(f)

    if key == "space":
        if _pub.get("estop"):
            _pub["estop"](1.0 if action == "down" else 0.0)
        if action == "down" and _pub.get("chassis"):
            _pub["chassis"](0, 0, 0)
        if action == "down" and _pub.get("lift"):
            _pub["lift"](0)
        return

    if key == "__chassis_zero__":
        _pub.get("chassis")(0, 0, 0) if _pub.get("chassis") else None
        return
    if key == "__lift_zero__":
        _pub.get("lift")(0) if _pub.get("lift") else None
        return

    sp = km.get("speed", {})
    ch = km.get("single", {}).get("chassis", {})
    for act, k in ch.items():
        if k != key:
            continue
        if action == "down":
            fwd = sp.get("chassis_forward", 0.3)
            strf = sp.get("chassis_strafe", 0.3)
            rot = sp.get("chassis_rotate", 30.0)
            m = {"forward":(fwd,0,0),"backward":(-fwd,0,0),"strafe_l":(0,-strf,0),
                 "strafe_r":(0,strf,0),"rotate_l":(0,0,rot),"rotate_r":(0,0,-rot)}
            _pub.get("chassis")(*m[act]) if _pub.get("chassis") else None
        else:
            _pub.get("chassis")(0, 0, 0) if _pub.get("chassis") else None
        return

    if mode == "single":
        wp = km.get("single", {}).get("weapon", {})
        for act, k in wp.items():
            if k == key and action == "down" and act in WEAPON_IDS:
                _pub.get("weapon")(WEAPON_IDS[act]) if _pub.get("weapon") else None
                return
        lf = km.get("single", {}).get("lift", {})
        for act, k in lf.items():
            if k == key:
                if action == "down":
                    v = sp.get("lift", 2.0)
                    m = {"up":v,"down":-v,"up_fast":v*2,"down_fast":-v*2}
                    _pub.get("lift")(m[act]) if _pub.get("lift") else None
                else:
                    _pub.get("lift")(0) if _pub.get("lift") else None
                return
        ks = km.get("single", {}).get("kfs", {})
        for act, k in ks.items():
            if k == key and action == "down" and act in KFS_IDS:
                dev, direc = KFS_IDS[act]
                _pub.get("kfs_pos")(dev, direc) if _pub.get("kfs_pos") else None
                return
    else:
        fl = km.get("auto", {}).get("flow", {})
        for act, k in fl.items():
            if k == key and action == "down" and act in FLOW_IDS:
                _pub.get("flow")(FLOW_IDS[act]) if _pub.get("flow") else None
                return
        zn = km.get("auto", {}).get("zone", {})
        for act, k in zn.items():
            if k == key and action == "down" and act in ZONE_IDS:
                _pub.get("zone")(ZONE_IDS[act]) if _pub.get("zone") else None
                return


class Handler(SimpleHTTPRequestHandler):
    def do_POST(self):
        if self.path == "/save":
            length = int(self.headers.get("Content-Length", 0))
            data = json.loads(self.rfile.read(length))
            with open(KEYMAP_PATH, "w", encoding="utf-8") as f:
                json.dump(data, f, indent=2, ensure_ascii=False)
            self._json({"ok": True})
        elif self.path == "/keystroke":
            length = int(self.headers.get("Content-Length", 0))
            data = json.loads(self.rfile.read(length))
            _publish_keystroke(data.get("key", ""), data.get("mode", "single"), data.get("action", ""))
            self._json({"ok": True})
        else:
            self.send_error(404)

    def do_GET(self):
        if self.path == "/load":
            with open(KEYMAP_PATH, "r", encoding="utf-8") as f:
                self._json(json.load(f))
        elif self.path == "/ping":
            self._json({"ok": True})
        elif self.path == "/":
            self.path = "/control.html"
            super().do_GET()
        else:
            super().do_GET()

    def _json(self, data):
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(json.dumps(data).encode("utf-8"))

    def log_message(self, *args):
        pass


def _start_ros():
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float32, Float32MultiArray
    rclpy.init()
    n = Node("web_control")
    ch = n.create_publisher(Float32MultiArray, "/chassis_cmd", 10)
    kf = n.create_publisher(Float32, "/kfs_cmd", 10)
    lf = n.create_publisher(Float32, "/lift_cmd", 10)
    fl = n.create_publisher(Float32, "/flow_cmd", 10)
    zn = n.create_publisher(Float32, "/zone_cmd", 10)
    es = n.create_publisher(Float32, "/pc_estop", 10)
    wp = n.create_publisher(Float32, "/weapon_cmd", 10)
    ks = n.create_publisher(Float32MultiArray, "/kfs_pos_cmd", 10)

    _pub["chassis"] = lambda vx, vy, vw: ch.publish(Float32MultiArray(data=[float(vx), float(vy), float(vw)]))
    _pub["kfs"] = lambda v: kf.publish(Float32(data=float(v)))
    _pub["lift"] = lambda v: lf.publish(Float32(data=float(v)))
    _pub["flow"] = lambda v: fl.publish(Float32(data=float(v)))
    _pub["zone"] = lambda v: zn.publish(Float32(data=float(v)))
    _pub["estop"] = lambda v: es.publish(Float32(data=float(v)))
    _pub["weapon"] = lambda d: wp.publish(Float32(data=float(d)))
    _pub["kfs_pos"] = lambda d, dr: ks.publish(Float32MultiArray(data=[float(d), float(dr)]))

    def ros_spin():
        while rclpy.ok():
            try:
                rclpy.spin_once(n, timeout_sec=0.05)
            except Exception:
                break
    threading.Thread(target=ros_spin, daemon=True).start()
    n.get_logger().info("web_control ROS2 connected")


if __name__ == "__main__":
    SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
    KEYMAP_PATH = os.path.join(SCRIPT_DIR, "keymap.json")
    os.chdir(SCRIPT_DIR)
    PORT = 17890

    try:
        _start_ros()
        print("web_control: ROS2 connected, starting HTTP server...")
    except Exception as e:
        print(f"web_control: ROS2 offline ({e}), web-only mode")

    server = HTTPServer(("0.0.0.0", PORT), Handler)
    print(f"✓ HTTP server started on port {PORT}")
    print(f"  控制面板: http://<NUC_IP>:{PORT}/control.html")
    print(f"  键位编辑: http://<NUC_IP>:{PORT}/editor.html")
    print(f"  按 Ctrl+C 退出")
    threading.Thread(target=server.serve_forever, daemon=True).start()

    try:
        while True:
            import time
            time.sleep(3600)
    except KeyboardInterrupt:
        server.shutdown()
        print("\nweb_control stopped")
