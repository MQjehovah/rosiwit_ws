#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
debug_bridge — 给浏览器 debug_hmi 用的 WebSocket + protobuf 数据桥。

为什么需要它：
  rosbridge 把 ROS 消息原样（JSON/CBOR）搬给浏览器。/map 一帧 4300 万格（43MB），
  /vanjee_points722 10MB/s，远程链路传不动；而且 rosbridge 是单线程的，序列化时
  整条链路都会停。

这个节点在机器人本地做三件事：
  1. 点云体素降采样（默认 5~10cm）
  2. 坐标量化成 int16 厘米
  3. zlib 压缩
然后用 **WebSocket 长连接 + protobuf 二进制帧** 推给浏览器。协议定义在
debug_bridge/proto/debug_bridge.proto，浏览器端用 protobufjs（JSON descriptor）解码。

接口（默认 0.0.0.0:9100）：
  - 连接后服务器发 Envelope{hello}
  - 客户端发 ClientCommand：
      SUBSCRIBE   stream=scan|points_2d, rate_hz, voxel, max_points
      UNSUBSCRIBE stream=...
      REQUEST     stream=grid|global_map|vanjee_points722|scan|points_2d
  - 服务器推 Envelope{cloud|grid|status|error}

Python 2.7（Melodic）没有 protobuf runtime，所以这里手工按 wire format 编码；
字段编号与 .proto 一一对应。设备上有 protoc 3.0，但 py2 没装 runtime。

用法：
  # 机器人上
  source /opt/ros/melodic/setup.bash
  python debug_bridge.py --port 9100

  # 本地联调（无 ROS）
  python3 debug_bridge.py --demo --port 9100
"""

from __future__ import print_function

import argparse
import math
import signal
import struct
import sys
import threading
import time
import zlib

from tornado import web, httpserver, ioloop, websocket

try:
    TEXT_TYPE = unicode  # noqa: F821  (Python 2)
except NameError:
    TEXT_TYPE = str  # Python 3

VERSION = "2"
PROTO_VERSION = 2

# Envelope oneof field numbers
ENV_HELLO = 1
ENV_GRID = 2
ENV_CLOUD = 3
ENV_STATUS = 4
ENV_ERROR = 5

ACTION_SUBSCRIBE = 0
ACTION_UNSUBSCRIBE = 1
ACTION_REQUEST = 2

COMP_NONE = 0
COMP_ZLIB = 1

STREAM_SCAN = "scan"
STREAM_POINTS2D = "points_2d"
STREAM_GRID = "grid"
STREAM_GLOBAL = "global_map"
STREAM_VANJEE = "vanjee_points722"

CLOUD_TOPICS = {
    STREAM_SCAN: "/scan_f_map",
    STREAM_POINTS2D: "/points_2d",
    STREAM_GLOBAL: "/global_map",
    STREAM_VANJEE: "/vanjee_points722",
}


def now():
    return time.time()


def log(msg):
    sys.stderr.write("[debug_bridge] %s\n" % msg)
    sys.stderr.flush()


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def clamp_int(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


# --------------------------------------------------------------------------- protobuf


def pb_varint(n):
    if n < 0:
        n += 1 << 64
    out = bytearray()
    while True:
        b = n & 0x7F
        n >>= 7
        if n:
            out.append(b | 0x80)
        else:
            out.append(b)
            return bytes(out)


def pb_key(field, wire):
    return pb_varint((field << 3) | wire)


def pb_uint(field, value):
    if not value:
        return b""  # proto3 默认值省略
    return pb_key(field, 0) + pb_varint(int(value))


def pb_bool(field, value):
    if not value:
        return b""
    return pb_key(field, 0) + b"\x01"


def pb_float(field, value):
    if not value:
        return b""
    return pb_key(field, 5) + struct.pack("<f", float(value))


def pb_double(field, value):
    if not value:
        return b""
    return pb_key(field, 1) + struct.pack("<d", float(value))


def pb_bytes(field, value):
    if not value:
        return b""
    return pb_key(field, 2) + pb_varint(len(value)) + value


def pb_string(field, value):
    if not value:
        return b""
    data = value.encode("utf-8")
    return pb_key(field, 2) + pb_varint(len(data)) + data


def pb_message(field, payload):
    if not payload:
        return b""
    return pb_key(field, 2) + pb_varint(len(payload)) + payload


def envelope(field, payload):
    return pb_message(field, payload)


def read_varint(buf, offset):
    shift = 0
    result = 0
    while True:
        b = bytearray(buf[offset:offset + 1])[0]
        offset += 1
        result |= (b & 0x7F) << shift
        if not (b & 0x80):
            return result, offset
        shift += 7


def pb_decode(buf):
    """Only what ClientCommand needs: varint/float/bytes."""
    out = {}
    i = 0
    n = len(buf)
    while i < n:
        key, i = read_varint(buf, i)
        field = key >> 3
        wire = key & 7
        if wire == 0:
            val, i = read_varint(buf, i)
            out[field] = val
        elif wire == 5:
            out[field] = struct.unpack_from("<f", buf, i)[0]
            i += 4
        elif wire == 1:
            out[field] = struct.unpack_from("<d", buf, i)[0]
            i += 8
        elif wire == 2:
            length, i = read_varint(buf, i)
            out[field] = buf[i:i + length]
            i += length
        else:
            break
    return out


def header(frame, stamp):
    return pb_string(1, frame or "map") + pb_double(2, stamp or 0.0)


def pose2d(x, y, theta):
    return pb_double(1, x) + pb_double(2, y) + pb_float(3, theta)


# --------------------------------------------------------------------------- store


class Store(object):
    def __init__(self):
        self.lock = threading.Lock()
        self.grid = None
        self.clouds = {}
        self.started = now()

    def set_grid(self, g):
        with self.lock:
            g = dict(g)
            g["recv"] = now()
            self.grid = g

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
SEQ_LOCK = threading.Lock()
SEQ = [0]


def next_seq():
    with SEQ_LOCK:
        SEQ[0] += 1
        return SEQ[0]


def voxelize(points, voxel, max_points):
    if (voxel is None or voxel <= 0) and not max_points:
        return points
    if not voxel or voxel <= 0:
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


def encode_cloud(stream, c, voxel, max_points):
    pts = voxelize(c["points"], voxel, max_points)
    raw = bytearray(len(pts) * 6)
    off = 0
    for p in pts:
        struct.pack_into(
            "<hhh",
            raw,
            off,
            clamp_int(int(round(p[0] * 100.0)), -32768, 32767),
            clamp_int(int(round(p[1] * 100.0)), -32768, 32767),
            clamp_int(int(round(p[2] * 100.0)), -32768, 32767),
        )
        off += 6
    body = zlib.compress(bytes(raw), 6)
    frame = (
        pb_message(1, header(c.get("frame"), c.get("stamp")))
        + pb_uint(2, len(pts))
        + pb_float(3, voxel or 0.0)
        + pb_uint(4, COMP_ZLIB)
        + pb_bytes(5, body)
        + pb_uint(6, next_seq())
        + pb_string(7, stream)
    )
    return envelope(ENV_CLOUD, frame)


def encode_grid(g):
    raw = g["data"]
    if hasattr(raw, "tobytes"):
        raw = raw.tobytes()
    elif hasattr(raw, "tostring"):
        raw = raw.tostring()
    elif not isinstance(raw, (bytes, bytearray)):
        raw = bytes(bytearray(raw))
    body = zlib.compress(raw, 6)
    frame = (
        pb_message(1, header(g.get("frame"), g.get("stamp")))
        + pb_uint(2, g["w"])
        + pb_uint(3, g["h"])
        + pb_float(4, g["res"])
        + pb_message(5, pose2d(g["ox"], g["oy"], g.get("oth") or 0.0))
        + pb_uint(6, COMP_ZLIB)
        + pb_bytes(7, body)
        + pb_uint(8, next_seq())
    )
    return envelope(ENV_GRID, frame)


def encode_status(stream, ok, message):
    frame = (
        pb_string(1, stream)
        + pb_bool(2, ok)
        + pb_string(3, message)
        + pb_uint(4, int(now() * 1000))
    )
    return envelope(ENV_STATUS, frame)


def encode_error(message):
    return envelope(ENV_ERROR, pb_string(1, message))


def encode_hello(streams):
    frame = pb_string(1, "debug_bridge") + pb_string(2, VERSION)
    for name, available, rate in streams:
        info = pb_string(1, name) + pb_bool(2, available) + pb_float(3, rate)
        frame += pb_message(3, info)
    return envelope(ENV_HELLO, frame)


# --------------------------------------------------------------------------- ROS


class OneShot(object):
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
    stride = int(math.ceil(float(n) / max_points)) if n > max_points else 1
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
    stride = int(math.ceil(float(len(pts)) / max_points)) if len(pts) > max_points else 1
    out = []
    for i in range(0, len(pts), stride):
        p = pts[i]
        if math.isnan(p.x) or math.isnan(p.y) or math.isnan(p.z):
            continue
        out.append((p.x, p.y, p.z))
    return out


class RosSource(object):
    def __init__(self):
        import rospy
        from sensor_msgs.msg import PointCloud, PointCloud2

        self.rospy = rospy
        self.PointCloud = PointCloud
        self.PointCloud2 = PointCloud2
        self.subs = {}
        self.subs_lock = threading.Lock()
        self.grid_lock = threading.Lock()

    def start(self):
        self._ensure_sub("/scan_f_map", self.PointCloud)

    def _ensure_sub(self, topic, cls):
        with self.subs_lock:
            if topic in self.subs:
                return
            if cls is self.PointCloud:
                def cb(msg):
                    STORE.set_cloud(topic, pc_points(msg), msg.header.frame_id, msg.header.stamp.to_sec())
            else:
                def cb(msg):
                    STORE.set_cloud(topic, pc2_points(msg), msg.header.frame_id, msg.header.stamp.to_sec())

            self.subs[topic] = self.rospy.Subscriber(topic, cls, cb, queue_size=1, buff_size=2 ** 22)

    def ensure_stream(self, stream):
        if stream == STREAM_POINTS2D:
            self._ensure_sub("/points_2d", self.PointCloud2)

    def latest(self, stream, max_age=2.0):
        topic = CLOUD_TOPICS.get(stream)
        if not topic:
            return None
        c = STORE.get_cloud(topic)
        if c is None:
            return None
        if max_age and now() - c["recv"] > max_age:
            return None
        return c

    def grid(self, refresh=False, timeout=8.0):
        with self.grid_lock:
            current = STORE.get_grid()
            if current is not None and not refresh and now() - current["recv"] < 5.0:
                return current
            from nav_msgs.msg import OccupancyGrid

            one = OneShot("/map", OccupancyGrid)
            msg = one.wait(timeout)
            if msg is None:
                return current
            w = int(msg.info.width)
            h = int(msg.info.height)
            try:
                from array import array

                data = array("b", msg.data)
            except Exception:
                data = bytearray(bytearray((v & 0xFF) for v in msg.data))
            o = msg.info.origin
            g = {
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
            STORE.set_grid(g)
            return g

    def cloud(self, stream, timeout=8.0):
        if stream in (STREAM_SCAN, STREAM_POINTS2D):
            self.ensure_stream(stream)
            started = now()
            while now() - started < timeout:
                c = self.latest(stream)
                if c is not None:
                    return c
                time.sleep(0.05)
            return self.latest(stream)

        topic = CLOUD_TOPICS.get(stream)
        if not topic:
            return None
        if stream == STREAM_VANJEE:
            cls = self.PointCloud2
            one = OneShot(topic, cls)
            msg = one.wait(timeout)
            if msg is None:
                return None
            pts = pc2_points(msg)
            frame = msg.header.frame_id or "map"
            stamp = msg.header.stamp.to_sec()
        elif stream == STREAM_GLOBAL:
            one = OneShot(topic, self.PointCloud2)
            msg = one.wait(timeout)
            if msg is None:
                return None
            pts = pc2_points(msg)
            frame = msg.header.frame_id or "map"
            stamp = msg.header.stamp.to_sec()
        else:
            return None
        c = {"points": pts, "frame": frame, "stamp": stamp}
        STORE.set_cloud(topic, pts, frame, stamp)
        return c


# --------------------------------------------------------------------------- demo


class DemoSource(object):
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
                if 40 <= x < w - 40 and 40 <= y < h - 40:
                    v = 0
                    if x in (40, w - 41) or y in (40, h - 41):
                        v = 100
                    if (x == 300 and 40 <= y < 380) or (x == 540 and 220 <= y < h - 40):
                        v = 100
                    if (y == 380 and 40 <= x < 300) or (y == 220 and 540 <= x < w - 40):
                        v = 100
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
                x = cx + r * math.cos(a)
                y = cy + r * math.sin(a)
                if 2.0 <= x <= 16.0 and 2.0 <= y <= 13.0:
                    pts.append((x, y, 0.25 + 0.15 * math.sin(a * 3.0)))
            STORE.set_cloud("/scan_f_map", pts, "map", now())
            time.sleep(0.1)

    def start(self):
        pass

    def ensure_stream(self, stream):
        pass

    def latest(self, stream, max_age=2.0):
        return STORE.get_cloud(CLOUD_TOPICS.get(stream, ""))

    def grid(self, refresh=False, timeout=6.0):
        g = dict(self._grid)
        g["stamp"] = now()
        STORE.set_grid(g)
        return STORE.get_grid()

    def cloud(self, stream, timeout=6.0):
        if stream in (STREAM_SCAN, STREAM_POINTS2D):
            c = STORE.get_cloud("/scan_f_map")
            return c
        if stream == STREAM_GLOBAL:
            return {"points": [], "frame": "map", "stamp": now()}
        if stream == STREAM_VANJEE:
            c = STORE.get_cloud("/scan_f_map")
            return c
        return None


# --------------------------------------------------------------------------- websocket


class Client(object):
    def __init__(self, handler):
        self.handler = handler
        self.subs = {}
        self.closed = False
        self.last_send = now()

    def send(self, payload):
        if self.closed:
            return
        LOOP.add_callback(self._write, payload)

    def _write(self, payload):
        if self.closed:
            return
        try:
            self.handler.write_message(payload, binary=True)
            self.last_send = now()
        except Exception:
            self.closed = True


CLIENTS = []
CLIENTS_LOCK = threading.Lock()
LOOP = None
SOURCE = None
STOP = threading.Event()
CAPTURE_LOCKS = {}


def source_streams():
    streams = []
    for name in (STREAM_SCAN, STREAM_POINTS2D, STREAM_GRID, STREAM_GLOBAL, STREAM_VANJEE):
        latest = SOURCE.latest(name) if name in (STREAM_SCAN, STREAM_POINTS2D) else None
        streams.append((name, latest is not None, 0.0))
    return streams


class BridgeWS(websocket.WebSocketHandler):
    def check_origin(self, origin):
        return True

    def open(self):
        self.client = Client(self)
        with CLIENTS_LOCK:
            CLIENTS.append(self.client)
        self.client.send(encode_hello(source_streams()))
        self.client.send(encode_status("bridge", True, "connected"))

    def on_message(self, message):
        if isinstance(message, TEXT_TYPE):
            self.client.send(encode_error("expected protobuf binary frame"))
            return
        try:
            fields = pb_decode(message)
        except Exception as e:
            self.client.send(encode_error("bad ClientCommand: %s" % e))
            return
        action = fields.get(1, ACTION_SUBSCRIBE)
        stream = fields.get(2, b"")
        if isinstance(stream, bytes):
            stream = stream.decode("utf-8", "replace")
        rate = fields.get(3, 0.0)
        voxel = fields.get(4, 0.0)
        max_points = fields.get(5, 0)
        refresh = bool(fields.get(6, 0))

        if action == ACTION_SUBSCRIBE:
            rate = min(20.0, max(0.5, rate or 4.0))
            voxel = min(1.0, max(0.0, voxel or 0.05))
            max_points = max_points or 200000
            self.client.subs[stream] = {
                "rate": rate,
                "voxel": voxel,
                "max": max_points,
                "last": 0.0,
            }
            if SOURCE is not None:
                SOURCE.ensure_stream(stream)
            self.client.send(encode_status(stream, True, "subscribed %.1fHz" % rate))
        elif action == ACTION_UNSUBSCRIBE:
            self.client.subs.pop(stream, None)
            self.client.send(encode_status(stream, True, "unsubscribed"))
        elif action == ACTION_REQUEST:
            self._request(self.client, stream, voxel, max_points, refresh)
        else:
            self.client.send(encode_error("unknown action %s" % action))

    def _request(self, client, stream, voxel, max_points, refresh):
        if stream == STREAM_GRID:
            voxel = 0.0
            max_points = 0
        elif voxel <= 0:
            voxel = 0.2 if stream == STREAM_GLOBAL else 0.05
        if not max_points:
            max_points = 500000 if stream == STREAM_GLOBAL else 200000

        def run():
            lock = CAPTURE_LOCKS.setdefault(stream, threading.Lock())
            if not lock.acquire(False):
                client.send(encode_status(stream, False, "busy"))
                return
            try:
                client.send(encode_status(stream, True, "capturing"))
                if stream == STREAM_GRID:
                    g = SOURCE.grid(refresh=refresh)
                    if g is None:
                        client.send(encode_error("no /map message"))
                    else:
                        client.send(encode_grid(g))
                else:
                    c = SOURCE.cloud(stream)
                    if c is None:
                        client.send(encode_error("no message on %s" % stream))
                    else:
                        client.send(encode_cloud(stream, c, voxel, max_points))
            except Exception as e:
                client.send(encode_error("%s: %s" % (stream, e)))
            finally:
                lock.release()

        t = threading.Thread(target=run)
        t.daemon = True
        t.start()

    def on_close(self):
        self.client.closed = True
        with CLIENTS_LOCK:
            if self.client in CLIENTS:
                CLIENTS.remove(self.client)


def publisher_loop():
    while not STOP.is_set():
        t0 = now()
        with CLIENTS_LOCK:
            clients = list(CLIENTS)
        for client in clients:
            if client.closed:
                continue
            for stream, sub in list(client.subs.items()):
                if stream not in (STREAM_SCAN, STREAM_POINTS2D):
                    continue
                interval = 1.0 / sub["rate"]
                if t0 - sub["last"] < interval:
                    continue
                c = SOURCE.latest(stream)
                if c is None:
                    continue
                sub["last"] = t0
                client.send(encode_cloud(stream, c, sub["voxel"], sub["max"]))
            # 空闲时每 10s 发一个 status，防止隧道把长连接判死
            if t0 - client.last_send > 10.0:
                client.send(encode_status("heartbeat", True, "alive"))
        STOP.wait(0.05)


def main(argv=None):
    global LOOP, SOURCE
    ap = argparse.ArgumentParser(description="rosiwit debug_bridge (WebSocket/protobuf)")
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=9100)
    ap.add_argument("--demo", action="store_true", help="不用 ROS，生成假数据")
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args(argv)

    if args.demo:
        SOURCE = DemoSource()
        log("DEMO mode, serving fake map + scan")
    else:
        try:
            import rospy
        except ImportError:
            log("rospy not found. Source /opt/ros/<distro>/setup.bash, or use --demo.")
            return 2
        rospy.init_node("debug_bridge", anonymous=False, disable_signals=True)
        SOURCE = RosSource()
        log("ROS mode, waiting for /scan_f_map ...")

    SOURCE.start()

    LOOP = ioloop.IOLoop.current()
    app = web.Application([(r"/", BridgeWS)])
    server = httpserver.HTTPServer(app)
    server.listen(args.port, address=args.host)
    log("listening on ws://%s:%d (protocol v%s)" % (args.host, args.port, VERSION))

    t = threading.Thread(target=publisher_loop)
    t.daemon = True
    t.start()

    def _stop(signum, frame):
        STOP.set()
        LOOP.add_callback(LOOP.stop)

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    try:
        LOOP.start()
    finally:
        STOP.set()
        server.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
