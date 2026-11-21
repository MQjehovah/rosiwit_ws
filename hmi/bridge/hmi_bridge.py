#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
hmi_bridge — 给浏览器 HMI 用的轻量数据桥。

为什么需要它：
  rosbridge 会把 ROS 消息原样（JSON/CBOR）搬给浏览器。/map 一帧 4300 万格（43MB），
  /vanjee_points722 10MB/s，远程隧道根本传不动，而且 rosbridge 是单线程的，序列化时
  整条链路都会停。

这个节点只做一件事：**在机器人本地把数据压缩到浏览器真正需要的量**，然后用 HTTP
返回二进制帧。浏览器用 fetch + DecompressionStream 解码，不依赖 rosbridge，也不碰
厂商接口。

接口（默认 0.0.0.0:9100）：
  GET /api/v1/health
  GET /api/v1/grid[?refresh=1]                      -> HMG1 帧（zlib 后的 int8 栅格）
  GET /api/v1/cloud?topic=scan_f_map&voxel=0.08&max=200000
                                                    -> HMC1 帧（zlib 后的 int16 厘米点）
  topic 支持: scan_f_map / curr_scan_point / global_map / points_2d / vanjee_points722

帧格式（小端）：
  grid : 'HMG1' | ver u8 | flags u8 | step u16 | res f32 | w u32 | h u32 |
         ox f64 | oy f64 | oth f32 | frame(u8 len+utf8) | stamp f64 | u32 len + zlib(int8[w*h])
  cloud: 'HMC1' | ver u8 | flags u8 | count u32 | frame(u8 len+utf8) | stamp f64 |
         u32 len + zlib(int16[count*3]，单位 cm)

用法：
  # 机器人上（ROS1，Python 2.7/3 均可，只依赖 rospy）
  python hmi_bridge.py --port 9100

  # 没有 ROS 时本地联调（生成假地图 + 旋转扫描）
  python3 hmi_bridge.py --demo --port 9100

  # 通过远程隧道访问时，把隧道目标端口改成 9100 即可，例如
  #   https://<host>/remote-terminal/tunnel/<SN>/<TOKEN>/127.0.0.1/9100/
"""

from __future__ import print_function

import argparse
import json
import math
import random
import struct
import sys
import threading
import time
import zlib

try:
    from http.server import BaseHTTPRequestHandler, HTTPServer
    from socketserver import ThreadingMixIn
    from urllib.parse import urlparse, parse_qs
except ImportError:  # Python 2.7
    from BaseHTTPServer import BaseHTTPRequestHandler, HTTPServer
    from SocketServer import ThreadingMixIn
    from urlparse import urlparse, parse_qs

VERSION = 1
GRID_MAGIC = b"HMG1"
CLOUD_MAGIC = b"HMC1"

# topic -> (python module, type name)
TOPIC_TYPES = {
    "scan_f_map": ("sensor_msgs.msg", "PointCloud"),
    "curr_scan_point": ("sensor_msgs.msg", "PointCloud2"),
    "global_map": ("sensor_msgs.msg", "PointCloud2"),
    "points_2d": ("sensor_msgs.msg", "PointCloud2"),
    "vanjee_points722": ("sensor_msgs.msg", "PointCloud2"),
}


def now():
    return time.time()


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def clamp_int(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


# --------------------------------------------------------------------------- store


class Store(object):
    """所有数据都在这里；HTTP 线程只读，ROS 回调只写。"""

    def __init__(self):
        self.lock = threading.Lock()
        self.grid = None  # dict(data=array('b'), w,h,res,ox,oy,oth,frame,stamp)
        self.grid_blob = None  # (stamp, payload)
        self.clouds = {}  # topic -> dict(points=[(x,y,z)], frame, stamp)
        self.started = now()

    def set_grid(self, g):
        with self.lock:
            g = dict(g)
            g["recv"] = now()
            self.grid = g
            self.grid_blob = None

    def get_grid(self):
        with self.lock:
            return self.grid

    def set_cloud(self, topic, points, frame, stamp):
        with self.lock:
            self.clouds[topic] = {"points": points, "frame": frame, "stamp": stamp, "recv": now()}

    def get_cloud(self, topic):
        with self.lock:
            return self.clouds.get(topic)


STORE = Store()


# --------------------------------------------------------------------------- encode


def encode_grid(g, step):
    raw = g["data"]
    if hasattr(raw, "tobytes"):
        raw = raw.tobytes()
    elif hasattr(raw, "tostring"):  # Python 2 array
        raw = raw.tostring()
    elif not isinstance(raw, (bytes, bytearray)):
        raw = bytes(bytearray(raw))
    body = zlib.compress(raw, 6)

    frame = g.get("frame") or "map"
    fb = frame.encode("utf-8", "replace")[:255]
    head = GRID_MAGIC + struct.pack("<BB", VERSION, 1)
    head += struct.pack("<H", step)
    head += struct.pack("<f", float(g["res"]))
    head += struct.pack("<II", int(g["w"]), int(g["h"]))
    head += struct.pack("<dd", float(g["ox"]), float(g["oy"]))
    head += struct.pack("<f", float(g.get("oth") or 0.0))
    head += struct.pack("<B", len(fb)) + fb
    head += struct.pack("<d", float(g.get("stamp") or 0.0))
    head += struct.pack("<I", len(body))
    return head + body


def encode_cloud(c, voxel, max_points):
    pts = voxelize(c["points"], voxel, max_points)
    buf = bytearray(len(pts) * 6)
    off = 0
    for p in pts:
        struct.pack_into(
            "<hhh",
            buf,
            off,
            clamp_int(int(round(p[0] * 100.0)), -32768, 32767),
            clamp_int(int(round(p[1] * 100.0)), -32768, 32767),
            clamp_int(int(round(p[2] * 100.0)), -32768, 32767),
        )
        off += 6
    body = zlib.compress(bytes(buf), 6)

    frame = c.get("frame") or "map"
    fb = frame.encode("utf-8", "replace")[:255]
    head = CLOUD_MAGIC + struct.pack("<BB", VERSION, 1)
    head += struct.pack("<I", len(pts))
    head += struct.pack("<B", len(fb)) + fb
    head += struct.pack("<d", float(c.get("stamp") or 0.0))
    head += struct.pack("<I", len(body))
    return head + body, len(pts)


def voxelize(points, voxel, max_points):
    if voxel is None or voxel <= 0:
        return points[: max_points or len(points)]
    seen = set()
    out = []
    inv = 1.0 / voxel
    limit = max_points if max_points and max_points > 0 else 10 ** 9
    for p in points:
        key = (int(p[0] * inv), int(p[1] * inv), int(p[2] * inv))
        if key in seen:
            continue
        seen.add(key)
        out.append(p)
        if len(out) >= limit:
            break
    return out


# --------------------------------------------------------------------------- ROS


class OneShot(object):
    """取一条消息就退订，避免长期背着 43MB 的 /map。"""

    def __init__(self, topic, msg_cls):
        self.event = threading.Event()
        self.msg = None
        self.sub = None
        if msg_cls is not None:
            import rospy

            self.sub = rospy.Subscriber(topic, msg_cls, self._cb, queue_size=1)

    def _cb(self, msg):
        if self.event.is_set():
            return
        self.msg = msg
        self.event.set()
        try:
            self.sub.unregister()
        except Exception:
            pass

    def wait(self, timeout):
        if self.event.wait(timeout):
            return self.msg
        try:
            if self.sub is not None:
                self.sub.unregister()
        except Exception:
            pass
        return None


def pc2_points(msg, max_points=1500000):
    """按 fields 偏移直接解析 PointCloud2，不做 Python 对象化。"""
    fields = dict((f.name, f) for f in msg.fields)
    if not all(k in fields for k in ("x", "y", "z")):
        return []
    step = msg.point_step
    data = msg.data
    fmt = {1: "b", 2: "B", 3: "h", 4: "H", 5: "i", 6: "I", 7: "f", 8: "d"}
    fx, fy, fz = fields["x"], fields["y"], fields["z"]
    if fx.datatype not in fmt or fy.datatype not in fmt or fz.datatype not in fmt:
        return []
    cx = "<" + fmt[fx.datatype]
    cy = "<" + fmt[fy.datatype]
    cz = "<" + fmt[fz.datatype]
    ox, oy, oz = fx.offset, fy.offset, fz.offset

    n = msg.width * msg.height
    if n > max_points:
        # 均匀抽稀，保证大图也能在几十毫秒内返回
        stride = int(math.ceil(float(n) / max_points))
    else:
        stride = 1

    out = []
    try:
        for i in range(0, n, stride):
            base = i * step
            x = struct.unpack_from(cx, data, base + ox)[0]
            y = struct.unpack_from(cy, data, base + oy)[0]
            z = struct.unpack_from(cz, data, base + oz)[0]
            if math.isnan(x) or math.isnan(y) or math.isnan(z):
                continue
            if math.isinf(x) or math.isinf(y) or math.isinf(z):
                continue
            out.append((x, y, z))
    except struct.error:
        pass
    return out


def pc_points(msg, max_points=600000):
    pts = msg.points
    stride = 1
    if len(pts) > max_points:
        stride = int(math.ceil(float(len(pts)) / max_points))
    out = []
    for i in range(0, len(pts), stride):
        p = pts[i]
        if math.isnan(p.x) or math.isnan(p.y) or math.isnan(p.z):
            continue
        out.append((p.x, p.y, p.z))
    return out


class RosSource(object):
    def __init__(self, scan_hz=10.0):
        import rospy
        from sensor_msgs.msg import PointCloud, PointCloud2

        self.rospy = rospy
        self.PointCloud = PointCloud
        self.PointCloud2 = PointCloud2
        self.scan_hz = scan_hz
        self.scan_sub = None
        self.lock = threading.Lock()
        self.grid_lock = threading.Lock()

    def start(self):
        self._subscribe_scan()

    def _subscribe_scan(self):
        import rospy

        def cb(msg):
            STORE.set_cloud("scan_f_map", pc_points(msg), msg.header.frame_id, msg.header.stamp.to_sec())

        self.scan_sub = rospy.Subscriber(
            "/scan_f_map", self.PointCloud, cb, queue_size=1, buff_size=2 ** 20
        )

    def grid(self, timeout=6.0, refresh=False):
        with self.grid_lock:
            current = STORE.get_grid()
            if current is not None and not refresh and now() - current["recv"] < 5.0:
                return current
            one = OneShot("/map", self._occupancy_type())
            msg = one.wait(timeout)
            if msg is None:
                return current
            g = self._grid_from_msg(msg)
            STORE.set_grid(g)
            return g

    def _occupancy_type(self):
        from nav_msgs.msg import OccupancyGrid

        return OccupancyGrid

    def _grid_from_msg(self, msg):
        w = int(msg.info.width)
        h = int(msg.info.height)
        try:
            from array import array

            data = array("b", msg.data)
        except Exception:
            data = bytearray(bytearray((v & 0xFF) for v in msg.data))
        o = msg.info.origin
        return {
            "data": data,
            "w": w,
            "h": h,
            "res": float(msg.info.resolution),
            "ox": float(o.position.x),
            "oy": float(o.position.y),
            "oth": yaw_from_quat(o.orientation),
            "frame": msg.header.frame_id or "map",
            "stamp": msg.header.stamp.to_sec(),
        }

    def cloud(self, topic, timeout=6.0):
        if topic == "scan_f_map":
            for _ in range(int(timeout * 20)):
                c = STORE.get_cloud("scan_f_map")
                if c is not None and now() - c["recv"] < 2.0:
                    return c
                time.sleep(0.05)
            return STORE.get_cloud("scan_f_map")

        if topic not in TOPIC_TYPES:
            return None
        module_name, type_name = TOPIC_TYPES[topic]
        mod = __import__(module_name, fromlist=[type_name])
        cls = getattr(mod, type_name)
        one = OneShot("/" + topic, cls)
        msg = one.wait(timeout)
        if msg is None:
            return None
        stamp = msg.header.stamp.to_sec()
        frame = msg.header.frame_id or "map"
        if type_name == "PointCloud":
            pts = pc_points(msg)
        else:
            pts = pc2_points(msg)
        c = {"points": pts, "frame": frame, "stamp": stamp}
        STORE.set_cloud(topic, pts, frame, stamp)
        return c


# --------------------------------------------------------------------------- demo


class DemoSource(object):
    """没有 ROS 时用于联调：一块带房间的假地图 + 绕圈扫描。"""

    def __init__(self):
        self.t0 = now()
        self.w = 800
        self.h = 600
        self.res = 0.05
        self.ox = -20.0
        self.oy = -15.0
        self._grid = self._make_grid()
        STORE.set_grid(self._grid)
        self.thread = threading.Thread(target=self._loop)
        self.thread.daemon = True
        self.thread.start()

    def _make_grid(self):
        w, h = self.w, self.h
        from array import array

        data = array("b", [-1]) * (w * h)
        for y in range(h):
            for x in range(w):
                v = -1
                # 外墙
                if 40 <= x < w - 40 and 40 <= y < h - 40:
                    v = 0
                    if x in (40, w - 41) or y in (40, h - 41):
                        v = 100
                    # 两个隔间
                    if (x == 300 and 40 <= y < 380) or (x == 540 and 220 <= y < h - 40):
                        v = 100
                    if (y == 380 and 40 <= x < 300) or (y == 220 and 540 <= x < w - 40):
                        v = 100
                    # 一些柱子
                    if 120 <= x <= 135 and 120 <= y <= 135:
                        v = 100
                data[y * w + x] = v
        return {
            "data": data,
            "w": w,
            "h": h,
            "res": self.res,
            "ox": self.ox,
            "oy": self.oy,
            "oth": 0.0,
            "frame": "map",
            "stamp": now(),
        }

    def _loop(self):
        while True:
            t = now() - self.t0
            cx = 10.0 + 4.0 * math.cos(t * 0.15)
            cy = 7.0 + 3.0 * math.sin(t * 0.15)
            pts = []
            for i in range(720):
                a = i * math.pi / 360.0 + t * 0.2
                r = 6.0 + 0.3 * math.sin(a * 5.0 + t)
                # 只在房间内命中
                x = cx + r * math.cos(a)
                y = cy + r * math.sin(a)
                if 2.0 <= x <= 16.0 and 2.0 <= y <= 13.0:
                    pts.append((x, y, 0.25 + 0.15 * math.sin(a * 3.0)))
            STORE.set_cloud("scan_f_map", pts, "map", now())
            time.sleep(0.15)

    def start(self):
        pass

    def grid(self, timeout=6.0, refresh=False):
        g = dict(self._grid)
        g["stamp"] = now()
        STORE.set_grid(g)
        return STORE.get_grid()

    def cloud(self, topic, timeout=6.0):
        c = STORE.get_cloud("scan_f_map")
        if c is None:
            return {"points": [], "frame": "map", "stamp": now()}
        return c


# --------------------------------------------------------------------------- HTTP


class Handler(BaseHTTPRequestHandler):
    source = None
    verbose = True

    def log_message(self, fmt, *args):
        if self.verbose:
            sys.stderr.write("%s %s\n" % (self.address_string(), fmt % args))

    def _cors(self):
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.send_header("Cache-Control", "no-store")

    def _json(self, code, obj):
        body = json.dumps(obj, ensure_ascii=False).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self._cors()
        self.end_headers()
        self.wfile.write(body)

    def _bin(self, payload, count, stamp):
        self.send_response(200)
        self.send_header("Content-Type", "application/octet-stream")
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("X-Count", str(count))
        self.send_header("X-Stamp", "%.3f" % stamp)
        self._cors()
        self.end_headers()
        self.wfile.write(payload)

    def do_OPTIONS(self):
        self.send_response(204)
        self._cors()
        self.send_header("Access-Control-Allow-Methods", "GET, OPTIONS")
        self.send_header("Content-Length", "0")
        self.end_headers()

    def do_GET(self):
        parsed = urlparse(self.path)
        path = parsed.path.rstrip("/") or "/"
        qs = parse_qs(parsed.query)

        if path == "/api/v1/health":
            grid = STORE.get_grid()
            clouds = {}
            for topic in STORE.clouds:
                c = STORE.get_cloud(topic)
                clouds[topic] = round(now() - c["recv"], 3) if c else None
            self._json(
                200,
                {
                    "ok": True,
                    "version": VERSION,
                    "uptime": round(now() - STORE.started, 1),
                    "grid": {"w": grid["w"], "h": grid["h"]} if grid else None,
                    "grid_age": round(now() - grid["recv"], 3) if grid else None,
                    "clouds": clouds,
                },
            )
            return

        if path == "/api/v1/grid":
            refresh = qs.get("refresh", ["0"])[0] in ("1", "true", "yes")
            try:
                g = self.source.grid(timeout=8.0, refresh=refresh)
                if g is None:
                    self._json(503, {"ok": False, "error": "no /map message yet"})
                    return
                payload = encode_grid(g, 1)
                self._bin(payload, g["w"] * g["h"], g.get("stamp") or 0.0)
            except Exception as e:
                self._json(500, {"ok": False, "error": str(e)})
            return

        if path == "/api/v1/cloud":
            topic = qs.get("topic", ["scan_f_map"])[0]
            try:
                voxel = float(qs.get("voxel", ["0.08"])[0])
            except ValueError:
                voxel = 0.08
            try:
                max_points = int(qs.get("max", ["200000"])[0])
            except ValueError:
                max_points = 200000
            try:
                wait = float(qs.get("wait", ["8"])[0])
            except ValueError:
                wait = 8.0
            try:
                c = self.source.cloud(topic, timeout=wait)
                if c is None:
                    self._json(503, {"ok": False, "error": "no message on /%s" % topic})
                    return
                payload, encoded = encode_cloud(c, voxel, max_points)
                self._bin(payload, encoded, c.get("stamp") or 0.0)
            except Exception as e:
                self._json(500, {"ok": False, "error": str(e)})
            return

        self._json(404, {"ok": False, "error": "not found", "path": path})


class ThreadingHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True
    allow_reuse_address = True


def main(argv=None):
    ap = argparse.ArgumentParser(description="rosiwit HMI data bridge")
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=9100)
    ap.add_argument("--demo", action="store_true", help="不用 ROS，生成假数据")
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args(argv)

    if args.demo:
        source = DemoSource()
        print("[hmi_bridge] DEMO mode, serving fake map + scan")
    else:
        try:
            import rospy
        except ImportError:
            print("rospy not found. Source /opt/ros/<distro>/setup.bash, or use --demo.", file=sys.stderr)
            return 2
        rospy.init_node("hmi_bridge", anonymous=False, disable_signals=True)
        source = RosSource()
        print("[hmi_bridge] ROS mode, waiting for /scan_f_map ...")

    source.start()
    Handler.source = source
    Handler.verbose = not args.quiet
    server = ThreadingHTTPServer((args.host, args.port), Handler)
    print("[hmi_bridge] listening on http://%s:%d (v%d)" % (args.host, args.port, VERSION))
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
