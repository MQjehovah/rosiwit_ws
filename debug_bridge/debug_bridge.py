#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
debug_bridge — debug_hmi 的唯一数据通道（WebSocket + protobuf）。

替代 rosbridge：TF、定位状态、日志、路径、LaserScan、足迹、点云、栅格、
话题列表/统计、服务调用、/initialpose 全部走这一个 9100 端口。

在机器人本地做：
  - 点云体素降采样（默认 5~10cm）
  - 坐标 int16 厘米量化 + zlib
  - 按客户端订阅的 rate_hz 主动推送
  - 服务/初始位姿/话题查询按请求执行

协议：debug_bridge/proto/debug_bridge.proto（包名 debug_bridge）
浏览器：protobufjs/light + 生成的 JSON descriptor

Python 2.7（Melodic）没有 protobuf runtime，这里按 wire format 手工编码；
字段编号与 .proto 一一对应。

用法：
  source /opt/ros/melodic/setup.bash
  python debug_bridge.py --port 9100

  # 本地联调（无 ROS）
  python3 debug_bridge.py --demo --port 9100
"""

from __future__ import print_function

import argparse
import collections
import json
import math
import os
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
    TEXT_TYPE = str

VERSION = "9"
# 单帧点云进入缓存前最多保留多少点（/global_map 有 600 万点，全量 Python tuple
# 会吃掉 1.5GB+ 内存和大量 CPU）。HMI 显示上限通常 30 万，100 万足够；
# 需要全量时用 DEBUG_BRIDGE_MAX_CLOUD_POINTS=0 关闭上限。
MAX_CLOUD_POINTS = int(os.environ.get("DEBUG_BRIDGE_MAX_CLOUD_POINTS", "1000000"))
# zlib 级别：1 比默认 6 快很多，包略大；隧道吃带宽的话可调到 3~6。
ZLIB_LEVEL = max(0, min(9, int(os.environ.get("DEBUG_BRIDGE_ZLIB_LEVEL", "1"))))

ENV_HELLO = 1
ENV_GRID = 2
ENV_CLOUD = 3
ENV_STATUS = 4
ENV_ERROR = 5
ENV_TF = 6
ENV_SLAM = 7
ENV_LOG = 8
ENV_PATH = 9
ENV_LASER = 10
ENV_POLYGON = 11
ENV_TOPICS = 12
ENV_STAT = 13
ENV_RAW = 14
ENV_SERVICE = 15
ENV_IMAGE = 16

ACTION_SUBSCRIBE = 0
ACTION_UNSUBSCRIBE = 1
ACTION_REQUEST = 2
ACTION_SET_INITIAL_POSE = 3
ACTION_CALL_SERVICE = 4
ACTION_GET_TOPICS = 5
ACTION_RAW_SUBSCRIBE = 6
ACTION_RAW_UNSUBSCRIBE = 7
ACTION_PUBLISH_TWIST = 8

COMP_NONE = 0
COMP_ZLIB = 1

# 内置流: 只有 TF / 定位状态 / 日志。其余全部按标准 ROS 话题动态订阅
# (stream 直接就是话题名，例如 "/curr_scan_point")。
STREAM_TF = "tf"
STREAM_STATUS = "status"
STREAM_LOGS = "logs"

DEFAULT_RATES = {
    STREAM_TF: 20.0,
    STREAM_STATUS: 10.0,
    STREAM_LOGS: 5.0,
}

STREAM_TOPICS = {
    STREAM_TF: ["/tf", "/tf_static"],
    STREAM_STATUS: ["/slam/info"],
    STREAM_LOGS: ["/rosout_agg"],
}

BUILTIN_STREAMS = (STREAM_TF, STREAM_STATUS, STREAM_LOGS)

# 话题类型 -> 归一化种类
KIND_CLOUD = "cloud"
KIND_GRID = "grid"
KIND_LASER = "laser"
KIND_PATH = "path"
KIND_POLYGON = "polygon"
KIND_IMAGE = "image"
KIND_RAW = "raw"

TYPE_KIND = {
    "sensor_msgs/PointCloud2": KIND_CLOUD,
    "sensor_msgs/PointCloud": KIND_CLOUD,
    "nav_msgs/OccupancyGrid": KIND_GRID,
    "sensor_msgs/LaserScan": KIND_LASER,
    "nav_msgs/Path": KIND_PATH,
    "geometry_msgs/PolygonStamped": KIND_POLYGON,
    "sensor_msgs/Image": KIND_IMAGE,
    "sensor_msgs/CompressedImage": KIND_IMAGE,
}

ALLOWED_SERVICES = set(
    [
        "/relocation",
        "/localization/relocalize",
        "/localization/set_initial_pose",
        "/slam/mode_switch",
        "/slam/dump_recorder",
        "/stop_map_operation",
        "/switch_map",
        "/localization/switch_map",
        "/update_map_operation",
    ]
)

MAX_RAW_JSON = 300000


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
        return b""
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
    if isinstance(value, bytes):
        data = value  # Python2 str (ROS 字符串) 已经是 UTF-8 字节
    elif isinstance(value, TEXT_TYPE):
        data = value.encode("utf-8")
    else:
        data = str(value).encode("utf-8")
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


def vector3(x, y, z):
    return pb_double(1, x) + pb_double(2, y) + pb_double(3, z)


def quaternion(x, y, z, w):
    return pb_double(1, x) + pb_double(2, y) + pb_double(3, z) + pb_double(4, w)


def encode_hello(streams):
    frame = pb_string(1, "debug_bridge") + pb_string(2, VERSION)
    for name, available, rate in streams:
        info = pb_string(1, name) + pb_bool(2, available) + pb_float(3, rate)
        frame += pb_message(3, info)
    return envelope(ENV_HELLO, frame)


def encode_status(stream, ok, message):
    frame = pb_string(1, stream) + pb_bool(2, ok) + pb_string(3, message)
    frame += pb_uint(4, int(now() * 1000))
    return envelope(ENV_STATUS, frame)


def encode_error(message):
    return envelope(ENV_ERROR, pb_string(1, message))


def encode_slam(s):
    frame = (
        pb_uint(1, s.get("slam_state", 0))
        + pb_uint(2, s.get("location_status", 0))
        + pb_float(3, s.get("matching_degree", 0.0))
        + pb_uint(4, s.get("update_request", 0))
        + pb_string(5, s.get("map_name", ""))
        + pb_double(6, s.get("map_area", 0.0))
        + pb_message(7, pose2d(s.get("x", 0.0), s.get("y", 0.0), s.get("theta", 0.0)))
        + pb_double(8, s.get("stamp", 0.0))
    )
    return envelope(ENV_SLAM, frame)


def encode_log(entry):
    frame = (
        pb_uint(1, int(entry.get("level", 0)))
        + pb_string(2, entry.get("name", ""))
        + pb_string(3, entry.get("msg", ""))
        + pb_double(4, entry.get("stamp", 0.0))
    )
    return envelope(ENV_LOG, frame)


def encode_tf(transforms, is_static):
    frame = b""
    for it in transforms:
        tr = pb_string(1, it["parent"]) + pb_string(2, it["child"])
        tr += pb_message(3, vector3(*it["t"]))
        tr += pb_message(4, quaternion(*it["q"]))
        tr += pb_double(5, it.get("stamp", 0.0))
        frame += pb_message(1, tr)
    if is_static:
        frame += pb_bool(2, True)
    return envelope(ENV_TF, frame)


def encode_path(stream, msg):
    frame = pb_message(1, header(msg.get("frame"), msg.get("stamp")))
    for p in msg.get("poses", []):
        pose = pose2d(p[0], p[1], p[2])
        item = pb_message(1, pose) + pb_double(2, p[3] if len(p) > 3 else 0.0)
        frame += pb_message(2, item)
    frame += pb_uint(3, msg.get("seq", 0))
    frame += pb_string(4, stream)
    return envelope(ENV_PATH, frame)


def encode_laser(msg, stream):
    ranges = msg["ranges"]
    try:
        raw = struct.pack("<%df" % len(ranges), *ranges)
    except Exception:
        raw = b""
    frame = (
        pb_message(1, header(msg.get("frame"), msg.get("stamp")))
        + pb_float(2, msg.get("angle_min", 0.0))
        + pb_float(3, msg.get("angle_max", 0.0))
        + pb_float(4, msg.get("angle_increment", 0.0))
        + pb_float(5, msg.get("range_min", 0.0))
        + pb_float(6, msg.get("range_max", 0.0))
        + pb_bytes(7, raw)
        + pb_string(8, stream)
    )
    return envelope(ENV_LASER, frame)


def encode_polygon(msg, stream):
    raw = struct.pack("<%df" % (len(msg["xy"])), *msg["xy"])
    frame = (
        pb_message(1, header(msg.get("frame"), msg.get("stamp")))
        + pb_bytes(2, raw)
        + pb_string(3, stream)
    )
    return envelope(ENV_POLYGON, frame)


def encode_image(msg, stream):
    frame = (
        pb_message(1, header(msg.get("frame"), msg.get("stamp")))
        + pb_uint(2, msg.get("w", 0))
        + pb_uint(3, msg.get("h", 0))
        + pb_string(4, msg.get("encoding", ""))
        + pb_uint(5, msg.get("step", 0))
        + pb_bytes(6, msg.get("data", b""))
        + pb_uint(7, next_seq())
        + pb_string(8, stream)
    )
    return envelope(ENV_IMAGE, frame)


def encode_topics(topics):
    frame = b""
    for name, typ in topics:
        info = pb_string(1, name) + pb_string(2, typ)
        frame += pb_message(1, info)
    return envelope(ENV_TOPICS, frame)


def encode_stat(topic, count, age_ms, size, total_bytes=0, rate_hz=0.0):
    frame = (
        pb_string(1, topic)
        + pb_uint(2, count)
        + pb_uint(3, age_ms)
        + pb_uint(4, size)
        + pb_uint(5, total_bytes)
        + pb_float(6, rate_hz)
    )
    return envelope(ENV_STAT, frame)


def encode_raw(topic, typ, text, stamp):
    frame = pb_string(1, topic) + pb_string(2, typ) + pb_string(3, text) + pb_double(4, stamp)
    return envelope(ENV_RAW, frame)


def encode_service(request_id, service, ok, text):
    frame = pb_string(1, request_id) + pb_string(2, service) + pb_bool(3, ok) + pb_string(4, text)
    return envelope(ENV_SERVICE, frame)


def encode_cloud(stream, c, voxel, max_points):
    pts = voxelize(c["points"], voxel, max_points)
    count = len(pts)
    iw = None
    if _np is not None and isinstance(pts, _np.ndarray):
        xyz = _np.asarray(pts[:, :3], dtype=_np.float64)
        xyz_cm = _np.clip(_np.round(xyz * 100.0), -32768, 32767).astype("<i2")
        body = zlib.compress(xyz_cm.tobytes(), ZLIB_LEVEL)
        if c.get("has_intensity"):
            if pts.shape[1] > 3:
                iv = _np.asarray(pts[:, 3], dtype=_np.float64)
                iv = _np.where(_np.isfinite(iv), iv, 0.0)
            else:
                iv = _np.zeros(count, dtype=_np.float64)
            iv = _np.clip(_np.round(iv), 0, 255).astype("u1")
            iw = zlib.compress(iv.tobytes(), ZLIB_LEVEL)
    else:
        raw = bytearray(count * 6)
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
        body = zlib.compress(bytes(raw), ZLIB_LEVEL)
        if c.get("has_intensity"):
            iwr = bytearray(count)
            for i, p in enumerate(pts):
                v = float(p[3]) if len(p) > 3 else 0.0
                iwr[i] = 0 if v < 0.0 else (255 if v > 255.0 else int(round(v)))
            iw = zlib.compress(bytes(iwr), ZLIB_LEVEL)
    frame = (
        pb_message(1, header(c.get("frame"), c.get("stamp")))
        + pb_uint(2, count)
        + pb_float(3, voxel or 0.0)
        + pb_uint(4, COMP_ZLIB)
        + pb_bytes(5, body)
        + pb_uint(6, next_seq())
        + pb_string(7, stream)
    )
    if iw is not None:
        frame += pb_bytes(10, iw)
    return envelope(ENV_CLOUD, frame)


def encode_grid(g, stream):
    raw = g["data"]
    if hasattr(raw, "tobytes"):
        raw = raw.tobytes()
    elif hasattr(raw, "tostring"):
        raw = raw.tostring()
    elif not isinstance(raw, (bytes, bytearray)):
        raw = bytes(bytearray(raw))
    body = zlib.compress(raw, ZLIB_LEVEL)
    frame = (
        pb_message(1, header(g.get("frame"), g.get("stamp")))
        + pb_uint(2, g["w"])
        + pb_uint(3, g["h"])
        + pb_float(4, g["res"])
        + pb_message(5, pose2d(g["ox"], g["oy"], g.get("oth") or 0.0))
        + pb_uint(6, COMP_ZLIB)
        + pb_bytes(7, body)
        + pb_uint(8, next_seq())
        + pb_string(9, stream)
    )
    return envelope(ENV_GRID, frame)


SEQ_LOCK = threading.Lock()
SEQ = [0]


def next_seq():
    with SEQ_LOCK:
        SEQ[0] += 1
        return SEQ[0]


def voxelize(points, voxel, max_points):
    if not voxel or voxel <= 0:
        return points[: max_points or len(points)]
    if _np is not None and isinstance(points, _np.ndarray):
        try:
            inv = 1.0 / voxel
            keys = _np.floor(points[:, :3] * inv).astype(_np.int64)
            _, idx = _np.unique(keys, axis=0, return_index=True)
            idx.sort()              # 保持原始点序，取每个体素第一次出现的点
            out = points[idx]
            if max_points and out.shape[0] > max_points:
                out = out[:max_points]
            return out
        except Exception:
            points = points.tolist()
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


# --------------------------------------------------------------------------- cache


class Cache(object):
    def __init__(self):
        self.lock = threading.Lock()
        self.tf_dynamic = {}
        self.tf_static = {}
        self.slam = None
        self.logs = collections.deque(maxlen=500)
        self.log_seq = 0
        self.laser = None
        self.paths = {"global_path": None, "local_path": None}
        self.footprint = None
        self.stats = {}  # topic -> {count, last, size}
        self.started = now()

    def touch(self, topic, size=0):
        with self.lock:
            st = self.stats.get(topic)
            if st is None:
                st = {"count": 0, "last": 0.0, "size": 0}
                self.stats[topic] = st
            st["count"] += 1
            st["last"] = now()
            st["size"] = size

    def stat(self, topic):
        with self.lock:
            st = self.stats.get(topic)
            if not st:
                return None
            return (st["count"], now() - st["last"], st["size"])

    def add_log(self, level, name, msg, stamp):
        with self.lock:
            self.log_seq += 1
            self.logs.append(
                {"seq": self.log_seq, "level": level, "name": name, "msg": msg, "stamp": stamp}
            )

    def logs_since(self, seq, limit=20):
        with self.lock:
            out = [e for e in self.logs if e["seq"] > seq]
        return out[:limit]

    def max_log_seq(self):
        with self.lock:
            return self.log_seq

    def set_tf(self, is_static, transforms):
        with self.lock:
            target = self.tf_static if is_static else self.tf_dynamic
            for it in transforms:
                target[it["child"]] = it

    def tf_snapshot(self, is_static):
        with self.lock:
            src = self.tf_static if is_static else self.tf_dynamic
            return list(src.values())


CACHE = Cache()
STORE_LOCK = threading.Lock()
CLOUDS = {}
GRID = {"data": None}
# 通用话题缓存: topic -> {"kind":..., "stamp":..., "recv":..., ...}
TOPIC_DATA = {}


def set_topic_data(topic, data):
    data = dict(data)
    data["recv"] = now()
    with STORE_LOCK:
        TOPIC_DATA[topic] = data
    CACHE.touch(topic)


def get_topic_data(topic):
    with STORE_LOCK:
        return TOPIC_DATA.get(topic)


def set_cloud(topic, points, frame, stamp, has_intensity=False):
    with STORE_LOCK:
        CLOUDS[topic] = {"points": points, "frame": frame, "stamp": stamp,
                         "recv": now(), "has_intensity": has_intensity}
    CACHE.touch(topic, len(points) * 7)


def get_cloud(topic):
    with STORE_LOCK:
        return CLOUDS.get(topic)


def set_grid(g):
    with STORE_LOCK:
        g = dict(g)
        g["recv"] = now()
        GRID["data"] = g
    CACHE.touch("/map", g["w"] * g["h"])


def get_grid():
    with STORE_LOCK:
        return GRID["data"]


# --------------------------------------------------------------------------- ROS


class OneShot(object):
    def __init__(self, topic, msg_cls):
        self.event = threading.Event()
        self.msg = None
        self.sub = None
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


_NP_FMT = {1: "i1", 2: "u1", 3: "i2", 4: "u2", 5: "i4", 6: "u4", 7: "f4", 8: "f8"}


def _np_dtype_str(code, big):
    s = _NP_FMT.get(code)
    if s is None:
        return None
    if s in ("i1", "u1"):
        return s
    return (">" if big else "<") + s


def pc2_points_np(msg, max_points=0):
    """PointCloud2 -> numpy (N,3) 或 (N,4=[x,y,z,intensity])，避免百万级 Python tuple。"""
    fields = dict((f.name, f) for f in msg.fields)
    empty = _np.zeros((0, 3), dtype=_np.float64)
    if not all(k in fields for k in ("x", "y", "z")):
        return empty, False
    specs = []
    for name in ("x", "y", "z"):
        f = fields[name]
        if f.count != 1:
            return empty, False
        s = _np_dtype_str(f.datatype, msg.is_bigendian)
        if s is None:
            return empty, False
        specs.append((name, s, f.offset))
    fi = fields.get("intensity") or fields.get("i")
    i_name = None
    if fi is not None and fi.count == 1:
        s = _np_dtype_str(fi.datatype, msg.is_bigendian)
        if s is not None:
            i_name = fi.name
            specs.append((i_name, s, fi.offset))
    step = int(msg.point_step)
    if step <= 0:
        return empty, i_name is not None
    dt = _np.dtype(
        {
            "names": [s[0] for s in specs],
            "formats": [s[1] for s in specs],
            "offsets": [s[2] for s in specs],
            "itemsize": step,
        }
    )
    n = int(msg.width) * int(msg.height)
    n = min(n, len(msg.data) // step)
    if n <= 0:
        return empty, i_name is not None
    arr = _np.frombuffer(msg.data, dtype=dt, count=n)
    if max_points and n > max_points:
        stride = int(math.ceil(float(n) / max_points))
        arr = arr[::stride]
    x = arr["x"].astype(_np.float64)
    y = arr["y"].astype(_np.float64)
    z = arr["z"].astype(_np.float64)
    mask = _np.isfinite(x) & _np.isfinite(y) & _np.isfinite(z)
    xyz = _np.column_stack((x[mask], y[mask], z[mask]))
    if i_name is not None:
        iv = arr[i_name].astype(_np.float64)
        iv = _np.where(_np.isfinite(iv), iv, 0.0)
        return _np.column_stack((xyz, iv[mask])), True
    return xyz, False


def pc2_points(msg, max_points=0):
    if _np is not None:
        try:
            return pc2_points_np(msg, max_points)
        except Exception:
            pass
    fields = dict((f.name, f) for f in msg.fields)
    if not all(k in fields for k in ("x", "y", "z")):
        return [], False
    step = msg.point_step
    data = msg.data
    fmt = {1: "b", 2: "B", 3: "h", 4: "H", 5: "i", 6: "I", 7: "f", 8: "d"}
    fx, fy, fz = fields["x"], fields["y"], fields["z"]
    if fx.datatype not in fmt or fy.datatype not in fmt or fz.datatype not in fmt:
        return [], False
    cx = "<" + fmt[fx.datatype]
    cy = "<" + fmt[fy.datatype]
    cz = "<" + fmt[fz.datatype]
    ox, oy, oz = fx.offset, fy.offset, fz.offset
    # 强度/标量: 有 intensity 字段就按 uint8 带上 (雷达强度、overlay 的持久度 w).
    fi = fields.get("intensity") or fields.get("i")
    ci = None
    oi = 0
    if fi is not None and fi.datatype in fmt:
        ci = "<" + fmt[fi.datatype]
        oi = fi.offset
    n = msg.width * msg.height
    stride = int(math.ceil(float(n) / max_points)) if max_points and n > max_points else 1
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
            if ci is None:
                out.append((x, y, z))
            else:
                iv = struct.unpack_from(ci, data, base + oi)[0]
                if math.isnan(iv) or math.isinf(iv):
                    iv = 0.0
                out.append((x, y, z, iv))
    except struct.error:
        pass
    return out, (ci is not None)


def pc_points(msg, max_points=0):
    pts = msg.points
    if _np is not None:
        try:
            n = len(pts)
            if not n:
                return _np.zeros((0, 3), dtype=_np.float64)
            stride = int(math.ceil(float(n) / max_points)) if max_points and n > max_points else 1
            arr = _np.asarray([[p.x, p.y, p.z] for p in pts[::stride]], dtype=_np.float64)
            return arr[_np.isfinite(arr).all(axis=1)]
        except Exception:
            pass
    stride = int(math.ceil(float(len(pts)) / max_points)) if max_points and len(pts) > max_points else 1
    out = []
    for i in range(0, len(pts), stride):
        p = pts[i]
        if math.isnan(p.x) or math.isnan(p.y) or math.isnan(p.z):
            continue
        out.append((p.x, p.y, p.z))
    return out


def grid_dict(msg):
    w = int(msg.info.width)
    h = int(msg.info.height)
    try:
        from array import array

        data = array("b", msg.data)
    except Exception:
        data = bytearray(bytearray((v & 0xFF) for v in msg.data))
    o = msg.info.origin
    return {
        "kind": KIND_GRID,
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


def laser_dict(msg):
    return {
        "kind": KIND_LASER,
        "frame": msg.header.frame_id,
        "stamp": msg.header.stamp.to_sec(),
        "angle_min": msg.angle_min,
        "angle_max": msg.angle_max,
        "angle_increment": msg.angle_increment,
        "range_min": msg.range_min,
        "range_max": msg.range_max,
        "ranges": list(msg.ranges),
    }


def path_dict(msg):
    poses = []
    for ps in msg.poses:
        p = ps.pose.position
        q = ps.pose.orientation
        poses.append((p.x, p.y, yaw_from_quat(q), p.z))
    return {
        "kind": KIND_PATH,
        "frame": msg.header.frame_id,
        "stamp": msg.header.stamp.to_sec(),
        "seq": msg.header.seq,
        "poses": poses,
    }


def polygon_dict(msg):
    xy = []
    for p in msg.polygon.points:
        xy.extend([p.x, p.y])
    return {
        "kind": KIND_POLYGON,
        "frame": msg.header.frame_id,
        "stamp": msg.header.stamp.to_sec(),
        "xy": xy,
    }


def image_dict(msg):
    frame = msg.header.frame_id
    stamp = msg.header.stamp.to_sec()
    if hasattr(msg, "format"):
        fmt = (msg.format or "").lower()
        encoding = "jpeg" if ("jpeg" in fmt or "jpg" in fmt) else ("png" if "png" in fmt else fmt)
        return {
            "kind": KIND_IMAGE,
            "frame": frame,
            "stamp": stamp,
            "w": 0,
            "h": 0,
            "encoding": encoding,
            "step": 0,
            "data": msg.data,
        }

    w = int(msg.width)
    h = int(msg.height)
    enc = (msg.encoding or "").lower()
    try:
        import numpy as np
        import cv2

        buf = np.frombuffer(msg.data, dtype=np.uint8)
        img = None
        if enc in ("bgr8", "rgb8"):
            img = buf.reshape(h, msg.step)[:, : w * 3].reshape(h, w, 3)
            if enc == "rgb8":
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        elif enc in ("mono8", "8uc1"):
            img = buf.reshape(h, msg.step)[:, :w]
        elif enc in ("mono16", "16uc1"):
            arr = np.frombuffer(msg.data, dtype=np.uint16).reshape(h, msg.step // 2)[:, :w]
            img = cv2.normalize(arr, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        elif enc in ("32fc1", "32fc1"):
            arr = np.frombuffer(msg.data, dtype=np.float32).reshape(h, msg.step // 4)[:, :w]
            img = cv2.normalize(arr, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        if img is not None:
            ok, jpg = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            if ok:
                return {
                    "kind": KIND_IMAGE,
                    "frame": frame,
                    "stamp": stamp,
                    "w": w,
                    "h": h,
                    "encoding": "jpeg",
                    "step": 0,
                    "data": jpg.tobytes(),
                }
    except Exception as e:
        log("image encode %s: %s" % (enc, e))
    return {
        "kind": KIND_IMAGE,
        "frame": frame,
        "stamp": stamp,
        "w": w,
        "h": h,
        "encoding": enc,
        "step": msg.step,
        "data": msg.data,
    }


def raw_dict(topic, typ, msg):
    from rosbridge_library.internal.message_conversion import extract_values

    text = json.dumps(extract_values(msg))
    if len(text) > MAX_RAW_JSON:
        text = text[:MAX_RAW_JSON] + "...(truncated)"
    stamp = 0.0
    try:
        stamp = msg.header.stamp.to_sec()
    except Exception:
        stamp = now()
    return {"kind": KIND_RAW, "type": typ, "json": text, "stamp": stamp}


# --------------------------------------------------------------------------- 点云观感修正
# 万集驱动把点云去旋转成"重力对齐"系, 但 frame_id 仍是 vanjee_lidar, TF 树里没有
# 这个 frame; 前端查不到 TF 时会把点云放在地图原点且不旋转 -> 画面上和地图差一个
# 大角度(约等于机器人 yaw). 这里统一转到 map 系: 平移用 lidar 的位置, 旋转用
# base_link 的姿态(重力对齐), 这样任何前端拿到的都是地图系点云.
try:
    import numpy as _np
except Exception:
    _np = None


def _quat_mat(q):
    x, y, z, w = float(q[0]), float(q[1]), float(q[2]), float(q[3])
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return [
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
        [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
    ]


def _mm(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def _mv(A, v):
    return [sum(A[i][k] * v[k] for k in range(3)) for i in range(3)]


def _chain_to_map(leaf):
    """leaf -> map 的 (R, t); 查不到返回 None."""
    edges = {}
    with CACHE.lock:
        for src in (CACHE.tf_static, CACHE.tf_dynamic):
            for e in src.values():
                edges[e["child"]] = e
    R = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    t = [0.0, 0.0, 0.0]
    cur = leaf
    for _ in range(8):
        if cur in ("map", "world"):
            return R, t
        e = edges.get(cur)
        if not e:
            return None
        Re = _quat_mat(e.get("q") or [0.0, 0.0, 0.0, 1.0])
        te = e.get("t") or [0.0, 0.0, 0.0]
        R = _mm(Re, R)                              # T = T_edge * T (子 -> 父)
        t = [a + b for a, b in zip(_mv(Re, t), te)]
        cur = e.get("parent") or ""
    return None


_CM_LAST = {}


def _cm_log(tag, msg):
    t = now()
    if t - _CM_LAST.get(tag, 0.0) < 2.0:
        return
    _CM_LAST[tag] = t
    log(msg)


def cloud_to_map(points, frame):
    """把"重力对齐但 frame 不在 TF 里"的点云转到 map 系."""
    if points is None or len(points) == 0 or not frame or frame in ("map", "world"):
        return points, frame
    if frame not in ("vanjee_lidar", "lidar"):
        return points, frame
    rt = _chain_to_map("lidar")
    rb = _chain_to_map("base_link")
    if not rt or not rb:
        _cm_log(frame, "cloud_to_map %s: TF not ready (lidar=%s base=%s)" % (
            frame, rt is not None, rb is not None))
        return points, frame
    R, t = rb[0], rt[1]   # 姿态跟随 base_link(水平), 位置用雷达
    _cm_log(frame, "cloud_to_map %s -> map pts=%d t=(%.2f,%.2f,%.2f)" % (
        frame, len(points), t[0], t[1], t[2]))
    if _np is not None:
        arr = _np.asarray(points, dtype=_np.float64)
        xyz = arr[:, :3].dot(_np.asarray(R).T) + _np.asarray(t)
        if arr.shape[1] > 3:
            xyz = _np.hstack([xyz, arr[:, 3:4]])   # 强度列原样保留
        return xyz, "map"
    out = []
    for p in points:
        q = (
            R[0][0] * p[0] + R[0][1] * p[1] + R[0][2] * p[2] + t[0],
            R[1][0] * p[0] + R[1][1] * p[1] + R[1][2] * p[2] + t[1],
            R[2][0] * p[0] + R[2][1] * p[1] + R[2][2] * p[2] + t[2],
        )
        out.append(q + (p[3],) if len(p) > 3 else q)
    return out, "map"


def normalize_topic(topic, typ, msg):
    kind = TYPE_KIND.get(typ)
    if kind == KIND_CLOUD:
        if typ == "sensor_msgs/PointCloud2":
            pts, has_i = pc2_points(msg, MAX_CLOUD_POINTS)
        else:
            pts = pc_points(msg, MAX_CLOUD_POINTS)
            has_i = False
        frame = msg.header.frame_id or "map"
        pts, frame = cloud_to_map(pts, frame)
        return {
            "kind": KIND_CLOUD,
            "points": pts,
            "frame": frame,
            "stamp": msg.header.stamp.to_sec(),
            "has_intensity": has_i,
        }
    if kind == KIND_GRID:
        return grid_dict(msg)
    if kind == KIND_LASER:
        return laser_dict(msg)
    if kind == KIND_PATH:
        return path_dict(msg)
    if kind == KIND_POLYGON:
        return polygon_dict(msg)
    if kind == KIND_IMAGE:
        return image_dict(msg)
    return raw_dict(topic, typ, msg)


class RosSource(object):
    def __init__(self):
        import rospy

        self.rospy = rospy
        self.subs = {}
        self.sub_specs = {}
        self.lock = threading.Lock()
        self.grid_lock = threading.Lock()
        self.topic_types = {}
        self.types_at = 0.0
        self.twist_pub = None

    def set_twist(self, linear, angular):
        import rospy
        from geometry_msgs.msg import Twist

        with self.lock:
            if self.twist_pub is None:
                self.twist_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
                time.sleep(0.15)
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.twist_pub.publish(msg)
        CACHE.touch("/cmd_vel", 32)
        return True

    # --- subscriptions -----------------------------------------------------

    def start(self):
        t = threading.Thread(target=self._watchdog)
        t.daemon = True
        t.start()

    def _sub(self, topic, cls, cb, buff_size=2 ** 22):
        with self.lock:
            if topic in self.subs:
                return
            self.subs[topic] = self.rospy.Subscriber(topic, cls, cb, queue_size=1, buff_size=buff_size)
            self.sub_specs[topic] = (cls, cb, buff_size)

    def _resubscribe(self, topic):
        """rospy 的 TCPROS 线程在节点重启时偶尔会异常退出，订阅从此收不到数据。
        重新创建一个 Subscriber（同 topic/类型/回调）即可恢复。"""
        with self.lock:
            sub = self.subs.pop(topic, None)
            spec = self.sub_specs.get(topic)
        if sub is not None:
            try:
                sub.unregister()
            except Exception:
                pass
        if not spec:
            return
        cls, cb, buff_size = spec
        with self.lock:
            self.subs[topic] = self.rospy.Subscriber(topic, cls, cb, queue_size=1, buff_size=buff_size)
        # 重置计时，避免新连接还在握手时下一轮 watchdog 又把它拆掉
        CACHE.touch(topic, 0)
        log("watchdog: resubscribed %s" % topic)

    def _watchdog(self):
        # 只盯持续高频的内置话题：/slam/info 10Hz、/tf 50Hz+。
        # /tf_static 是 latched、/rosout_agg 可能长时间安静，不能按“无数据”重建。
        limits = {"/slam/info": 4.0, "/tf": 4.0}
        while not STOP.is_set():
            STOP.wait(2.0)
            if STOP.is_set():
                break
            try:
                for topic, limit in limits.items():
                    with self.lock:
                        has_sub = topic in self.subs
                    if not has_sub:
                        continue
                    stat = CACHE.stat(topic)
                    if not stat or stat[1] <= limit:
                        continue
                    self._resubscribe(topic)
            except Exception as e:
                log("watchdog: %s" % e)

    def release_topic(self, topic):
        """没有客户端再需要这个普通话题时，取消 ROS 订阅并丢掉缓存。
        否则点过一次 /global_map 就会永久占着 600 万点缓存 + 持续归一化。"""
        if topic in BUILTIN_STREAMS or topic in ("/tf", "/tf_static", "/slam/info", "/rosout_agg"):
            return
        with CLIENTS_LOCK:
            clients = list(CLIENTS)
        for c in clients:
            if topic in c.subs or topic in c.raw_subs:
                return
        with self.lock:
            sub = self.subs.pop(topic, None)
            self.sub_specs.pop(topic, None)
        if sub is not None:
            try:
                sub.unregister()
            except Exception:
                pass
        with STORE_LOCK:
            TOPIC_DATA.pop(topic, None)
        log("released %s (no clients)" % topic)

    def ensure_stream(self, stream):
        from sensor_msgs.msg import PointCloud2, LaserScan
        from nav_msgs.msg import Path
        from geometry_msgs.msg import PolygonStamped

        if stream == STREAM_TF:
            from tf2_msgs.msg import TFMessage

            self._sub("/tf", TFMessage, lambda m: self._on_tf(m, False), 2 ** 20)
            self._sub("/tf_static", TFMessage, lambda m: self._on_tf(m, True), 2 ** 20)
        elif stream == STREAM_STATUS:
            from xzrobot_ros_dependency.msg import SlamInfo

            self._sub("/slam/info", SlamInfo, self._on_slam)
        elif stream == STREAM_LOGS:
            from rosgraph_msgs.msg import Log

            self._sub("/rosout_agg", Log, self._on_log)
        elif stream.startswith("/"):
            self.ensure_topic(stream)

    def ensure_topic(self, topic):
        """按真实消息类型动态订阅任意话题，缓存归一化后的数据。"""
        with self.lock:
            if topic in self.subs:
                return
        typ = self.type_of(topic)
        if not typ:
            raise RuntimeError("topic not found: %s" % topic)
        from roslib.message import get_message_class

        cls = get_message_class(typ)
        if cls is None:
            raise RuntimeError("cannot load message class for %s" % typ)
        with self.lock:
            if topic in self.subs:
                return
            self.subs[topic] = self.rospy.Subscriber(
                topic,
                cls,
                lambda m: self._on_topic(topic, typ, m),
                queue_size=1,
                buff_size=2 ** 24,
            )
            self.sub_specs[topic] = (cls, lambda m: self._on_topic(topic, typ, m), 2 ** 24)

    def topic_data(self, topic):
        return get_topic_data(topic)

    def request_topic(self, topic, timeout):
        """等一帧（latched 话题会立刻返回），归一化后写入缓存。"""
        self.ensure_topic(topic)
        typ = self.type_of(topic)
        from roslib.message import get_message_class

        cls = get_message_class(typ)
        msg = OneShot(topic, cls).wait(timeout)
        if msg is None:
            return get_topic_data(topic)
        data = normalize_topic(topic, typ, msg)
        set_topic_data(topic, data)
        return get_topic_data(topic)

    # --- callbacks ---------------------------------------------------------

    def _on_tf(self, msg, is_static):
        items = []
        for tr in msg.transforms:
            items.append(
                {
                    "parent": tr.header.frame_id,
                    "child": tr.child_frame_id,
                    "t": (
                        tr.transform.translation.x,
                        tr.transform.translation.y,
                        tr.transform.translation.z,
                    ),
                    "q": (
                        tr.transform.rotation.x,
                        tr.transform.rotation.y,
                        tr.transform.rotation.z,
                        tr.transform.rotation.w,
                    ),
                    "stamp": tr.header.stamp.to_sec(),
                }
            )
        CACHE.set_tf(is_static, items)
        CACHE.touch("/tf_static" if is_static else "/tf", 64 * len(items))

    def _on_slam(self, msg):
        CACHE.slam = {
            "slam_state": msg.slam_state,
            "location_status": msg.location_status,
            "matching_degree": msg.matching_degree,
            "update_request": msg.update_request,
            "map_name": msg.map_name,
            "map_area": msg.map_area,
            "x": msg.robot_pose.x,
            "y": msg.robot_pose.y,
            "theta": msg.robot_pose.theta,
            "stamp": msg.header.stamp.to_sec() if hasattr(msg, "header") else now(),
        }
        if not CACHE.slam["stamp"]:
            CACHE.slam["stamp"] = now()
        CACHE.touch("/slam/info", 64)

    def _on_topic(self, topic, typ, msg):
        try:
            data = normalize_topic(topic, typ, msg)
            set_topic_data(topic, data)
        except Exception as e:
            log("normalize %s (%s): %s" % (topic, typ, e))

    def _on_log(self, msg):
        CACHE.add_log(msg.level, msg.name, msg.msg, msg.header.stamp.to_sec() or now())
        CACHE.touch("/rosout_agg", len(msg.msg))

    # --- one-shot ----------------------------------------------------------

    def grid(self, refresh=False, timeout=8.0):
        return self.request_topic("/map", timeout)

    # --- commands ----------------------------------------------------------

    def set_initial_pose(self, x, y, yaw):
        import rospy
        from geometry_msgs.msg import PoseWithCovarianceStamped

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = rospy.Time.now()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.0685
        pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1)
        time.sleep(0.2)
        pub.publish(msg)
        CACHE.touch("/initialpose", 128)
        return True

    def call_service(self, name, args_json):
        import rosservice
        from rospy import ServiceProxy
        from rosbridge_library.internal.message_conversion import populate_instance, extract_values

        if name not in ALLOWED_SERVICES:
            raise RuntimeError("service not allowed: %s" % name)
        args = json.loads(args_json or "{}")
        cls = rosservice.get_service_class_by_name(name)
        req = populate_instance(args, cls._request_class())
        resp = ServiceProxy(name, cls)(req)
        return extract_values(resp)

    def get_topics(self):
        code, msg, state = self.rospy.get_master().getTopicTypes()
        if code != 1:
            raise RuntimeError("getTopicTypes failed: %s" % msg)
        return state

    def type_of(self, topic):
        t = now()
        if t - self.types_at > 5.0:
            code, msg, state = self.rospy.get_master().getTopicTypes()
            if code == 1:
                self.topic_types = dict(state)
                self.types_at = t
        return self.topic_types.get(topic)

    def raw_subscribe(self, topic, cb):
        typ = self.type_of(topic)
        if not typ:
            raise RuntimeError("topic not found: %s" % topic)
        from roslib.message import get_message_class

        cls = get_message_class(typ)
        if cls is None:
            raise RuntimeError("cannot load message class for %s" % typ)
        import rospy

        def _cb(msg):
            cb(topic, typ, msg)

        return rospy.Subscriber(topic, cls, _cb, queue_size=1, buff_size=2 ** 22)


# --------------------------------------------------------------------------- demo


class DemoSource(object):
    def __init__(self):
        self.t0 = now()
        self.w = 800
        self.h = 600
        self.res = 0.05
        self.ox = -20.0
        self.oy = -15.0
        g = self._make_grid()
        set_grid(g)
        set_topic_data("/map", g)
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
            "kind": KIND_GRID,
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
            x = 10.0 + 4.0 * math.cos(t * 0.15)
            y = 7.0 + 3.0 * math.sin(t * 0.15)
            yaw = t * 0.15 + math.pi / 2
            q = (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))
            CACHE.set_tf(
                False,
                [
                    {
                        "parent": "map",
                        "child": "base_link",
                        "t": (x, y, 0.0),
                        "q": q,
                        "stamp": now(),
                    }
                ],
            )
            CACHE.set_tf(
                True,
                [
                    {
                        "parent": "base_link",
                        "child": "lidar",
                        "t": (0.4, 0.0, 0.9),
                        "q": (0.0, 0.0, 0.0, 1.0),
                        "stamp": now(),
                    }
                ],
            )
            CACHE.touch("/tf", 64)
            CACHE.touch("/tf_static", 64)
            CACHE.slam = {
                "slam_state": 1,
                "location_status": 0,
                "matching_degree": 0.92,
                "update_request": 0,
                "map_name": "demo",
                "map_area": 107733.0,
                "x": x,
                "y": y,
                "theta": yaw,
                "stamp": now(),
            }
            CACHE.touch("/slam/info", 64)
            ranges = []
            for i in range(720):
                a = i * math.pi / 360.0
                ranges.append(6.0 + 0.5 * math.sin(a * 3.0 + t))
            set_topic_data(
                "/scan",
                {
                    "kind": KIND_LASER,
                    "frame": "map",
                    "stamp": now(),
                    "angle_min": 0.0,
                    "angle_max": math.pi * 2,
                    "angle_increment": math.pi / 360.0,
                    "range_min": 0.1,
                    "range_max": 20.0,
                    "ranges": ranges,
                },
            )
            poses = []
            for i in range(120):
                a = t * 0.15 + i * 0.05
                poses.append((10.0 + 4.0 * math.cos(a), 7.0 + 3.0 * math.sin(a), a, 0.0))
            set_topic_data(
                "/global_path",
                {
                    "kind": KIND_PATH,
                    "frame": "map",
                    "stamp": now(),
                    "seq": int(t * 10),
                    "poses": poses,
                },
            )
            xy = [-0.5, -0.35, 0.5, -0.35, 0.5, 0.35, -0.5, 0.35]
            set_topic_data(
                "/robot_footprint",
                {"kind": KIND_POLYGON, "frame": "base_link", "stamp": now(), "xy": xy},
            )
            pts = []
            for i in range(720):
                a = i * math.pi / 360.0 + t * 0.2
                r = 5.0 + 0.3 * math.sin(a * 5.0 + t)
                px = x + r * math.cos(a)
                py = y + r * math.sin(a)
                if 2.0 <= px <= 16.0 and 2.0 <= py <= 13.0:
                    pts.append((px, py, 0.25 + 0.15 * math.sin(a * 3.0)))
            set_topic_data(
                "/curr_scan_point", {"kind": KIND_CLOUD, "points": pts, "frame": "map", "stamp": now()}
            )
            set_topic_data(
                "/global_map", {"kind": KIND_CLOUD, "points": pts, "frame": "map", "stamp": now()}
            )
            if int(t) % 10 == 0:
                CACHE.add_log(2, "/demo", "demo heartbeat %d" % int(t), now())
            time.sleep(0.1)

    def start(self):
        pass

    def ensure_stream(self, stream):
        pass

    def ensure_topic(self, topic):
        pass

    def topic_data(self, topic):
        return get_topic_data(topic)

    def request_topic(self, topic, timeout):
        return get_topic_data(topic)

    def grid(self, refresh=False, timeout=6.0):
        return get_topic_data("/map")

    def set_initial_pose(self, x, y, yaw):
        CACHE.add_log(2, "/demo", "initial pose %.2f %.2f %.2f" % (x, y, yaw), now())
        return True

    def set_twist(self, linear, angular):
        CACHE.add_log(2, "/demo", "twist %.2f %.2f" % (linear, angular), now())
        return True

    def call_service(self, name, args_json):
        return {"ok": True, "service": name, "args": json.loads(args_json or "{}")}

    def get_topics(self):
        names = [
            "/tf",
            "/tf_static",
            "/slam/info",
            "/rosout_agg",
            "/curr_scan_point",
            "/scan",
            "/points_2d",
            "/global_path",
            "/local_path",
            "/robot_footprint",
            "/map",
            "/global_map",
            "/vanjee_points722",
            "/initialpose",
        ]
        return [[n, self.type_of(n)] for n in names]

    def type_of(self, topic):
        if topic in ("/curr_scan_point", "/global_map", "/points_2d", "/vanjee_points722"):
            return "sensor_msgs/PointCloud2"
        if topic == "/map":
            return "nav_msgs/OccupancyGrid"
        if topic == "/scan":
            return "sensor_msgs/LaserScan"
        if topic in ("/global_path", "/local_path"):
            return "nav_msgs/Path"
        if topic == "/robot_footprint":
            return "geometry_msgs/PolygonStamped"
        if topic == "/slam/info":
            return "xzrobot_ros_dependency/SlamInfo"
        if topic == "/rosout_agg":
            return "rosgraph_msgs/Log"
        return "std_msgs/String"

    def raw_subscribe(self, topic, cb):
        raise RuntimeError("raw subscribe not available in demo mode")


# --------------------------------------------------------------------------- ws


class Client(object):
    def __init__(self, handler):
        self.handler = handler
        self.subs = {}
        self.closed = False
        self.last_send = now()
        self.log_seq = CACHE.max_log_seq()
        self.raw_subs = {}
        self.raw_last = {}
        self.last_stats = 0.0
        self.sent = {}  # topic -> {count, total, size, last}
        self.stat_prev = {}
        self.last_twist = 0.0
        self.twist_active = False

    def send(self, payload):
        if self.closed:
            return
        LOOP.add_callback(self._write, payload)

    def send_topic(self, topic, payload):
        if topic:
            st = self.sent.get(topic)
            if st is None:
                st = {"count": 0, "total": 0, "size": 0, "last": 0.0}
                self.sent[topic] = st
            st["count"] += 1
            st["total"] += len(payload)
            st["size"] = len(payload)
            st["last"] = now()
        self.send(payload)

    def _write(self, payload):
        if self.closed:
            return
        try:
            self.handler.write_message(payload, binary=True)
            self.last_send = now()
        except Exception:
            self.closed = True

    def close_raw(self):
        for sub in self.raw_subs.values():
            try:
                sub.unregister()
            except Exception:
                pass
        self.raw_subs.clear()


CLIENTS = []
CLIENTS_LOCK = threading.Lock()
LOOP = None
SOURCE = None
STOP = threading.Event()
CAPTURE_LOCKS = {}


def stream_available(stream):
    if stream == STREAM_TF:
        return bool(CACHE.tf_snapshot(False) or CACHE.tf_snapshot(True))
    if stream == STREAM_STATUS:
        return CACHE.slam is not None
    if stream == STREAM_LOGS:
        return CACHE.max_log_seq() > 0
    return False


def hello_streams():
    names = [STREAM_TF, STREAM_STATUS, STREAM_LOGS]
    return [(n, stream_available(n), DEFAULT_RATES.get(n, 0.0)) for n in names]


def encode_topic_frame(topic, data, sub):
    kind = data.get("kind")
    if kind == KIND_CLOUD:
        return encode_cloud(topic, data, sub.get("voxel", 0.0), sub.get("max", 0))
    if kind == KIND_GRID:
        return encode_grid(data, topic)
    if kind == KIND_LASER:
        return encode_laser(data, topic)
    if kind == KIND_PATH:
        return encode_path(topic, data)
    if kind == KIND_POLYGON:
        return encode_polygon(data, topic)
    if kind == KIND_IMAGE:
        return encode_image(data, topic)
    if kind == KIND_RAW:
        return encode_raw(topic, data.get("type", ""), data.get("json", ""), data.get("stamp", 0.0))
    return None


def frames_for_stream(stream, sub, client):
    out = []
    if stream == STREAM_TF:
        dyn = CACHE.tf_snapshot(False)
        if dyn:
            out.append(encode_tf(dyn, False))
        if now() - sub.get("last_static", 0.0) > 2.0:
            sta = CACHE.tf_snapshot(True)
            if sta:
                out.append(encode_tf(sta, True))
                sub["last_static"] = now()
    elif stream == STREAM_STATUS:
        if CACHE.slam:
            out.append(encode_slam(CACHE.slam))
    elif stream == STREAM_LOGS:
        entries = CACHE.logs_since(client.log_seq, 20)
        for e in entries:
            client.log_seq = e["seq"]
            out.append(encode_log(e))
    elif stream.startswith("/"):
        data = get_topic_data(stream)
        if not data or now() - data["recv"] > 10.0:
            return out
        # 去重键用 recv (桥接收到 ROS 消息的本地时间), 不用 header.stamp:
        # /curr_scan_point 这类话题的 header.stamp 可能冻结(定位位姿时间不更新),
        # 只比 stamp 会把之后所有新帧都丢掉, HMI 看起来就是 0Hz 不刷新。
        recv = data.get("recv", 0.0)
        if sub.get("last_recv") == recv:
            return out
        frame = encode_topic_frame(stream, data, sub)
        if frame is not None:
            sub["last_recv"] = recv
            out.append(frame)
    return out


class BridgeWS(websocket.WebSocketHandler):
    def check_origin(self, origin):
        return True

    def open(self):
        self.client = Client(self)
        with CLIENTS_LOCK:
            CLIENTS.append(self.client)
        self.client.send(encode_hello(hello_streams()))
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
        x = fields.get(7, 0.0)
        y = fields.get(8, 0.0)
        yaw = fields.get(9, 0.0)
        service = fields.get(10, b"")
        if isinstance(service, bytes):
            service = service.decode("utf-8", "replace")
        args_json = fields.get(11, b"")
        if isinstance(args_json, bytes):
            args_json = args_json.decode("utf-8", "replace")
        request_id = fields.get(12, b"")
        if isinstance(request_id, bytes):
            request_id = request_id.decode("utf-8", "replace")
        topic = fields.get(13, b"")
        if isinstance(topic, bytes):
            topic = topic.decode("utf-8", "replace")
        manual = bool(fields.get(15, 0))
        linear_x = fields.get(16, 0.0)
        angular_z = fields.get(17, 0.0)

        try:
            if action == ACTION_SUBSCRIBE:
                is_topic = stream.startswith("/")
                SOURCE.ensure_stream(stream)
                if is_topic:
                    typ = SOURCE.type_of(stream)
                    default_voxel = 0.05 if TYPE_KIND.get(typ) == KIND_CLOUD else 0.0
                    default_rate = 2.0
                else:
                    default_voxel = 0.0
                    default_rate = DEFAULT_RATES.get(stream, 5.0)
                rate = 0.0 if manual else min(30.0, max(0.5, rate or default_rate))
                self.client.subs[stream] = {
                    "rate": rate,
                    "voxel": min(1.0, max(0.0, voxel if voxel > 0 else default_voxel)),
                    "max": max_points or 0,
                    "last": 0.0,
                    "last_static": 0.0,
                    "last_recv": None,
                }
                self.client.send(
                    encode_status(stream, True, "manual" if manual else "subscribed %.1fHz" % rate)
                )
            elif action == ACTION_UNSUBSCRIBE:
                self.client.subs.pop(stream, None)
                self.client.send(encode_status(stream, True, "unsubscribed"))
                try:
                    SOURCE.release_topic(stream)
                except Exception as e:
                    log("release %s: %s" % (stream, e))
            elif action == ACTION_REQUEST:
                self._request(self.client, stream, voxel, max_points, refresh)
            elif action == ACTION_SET_INITIAL_POSE:
                SOURCE.set_initial_pose(x, y, yaw)
                self.client.send(encode_status("initialpose", True, "published"))
            elif action == ACTION_PUBLISH_TWIST:
                SOURCE.set_twist(linear_x, angular_z)
                self.client.last_twist = now()
                self.client.twist_active = abs(linear_x) > 1e-4 or abs(angular_z) > 1e-4
            elif action == ACTION_CALL_SERVICE:
                self._service(self.client, service, args_json, request_id)
            elif action == ACTION_GET_TOPICS:
                self._topics(self.client)
            elif action == ACTION_RAW_SUBSCRIBE:
                self._raw_subscribe(self.client, topic, rate)
            elif action == ACTION_RAW_UNSUBSCRIBE:
                sub = self.client.raw_subs.pop(topic, None)
                if sub is not None:
                    try:
                        sub.unregister()
                    except Exception:
                        pass
            else:
                self.client.send(encode_error("unknown action %s" % action))
        except Exception as e:
            self.client.send(encode_error("%s" % e))

    def _request(self, client, stream, voxel, max_points, refresh):
        if stream == "grid":
            stream = "/map"
        if not stream.startswith("/"):
            client.send(encode_error("request needs a topic name: %s" % stream))
            return

        def run():
            lock = CAPTURE_LOCKS.setdefault(stream, threading.Lock())
            if not lock.acquire(False):
                client.send(encode_status(stream, False, "busy"))
                return
            try:
                client.send(encode_status(stream, True, "capturing"))
                typ = SOURCE.type_of(stream)
                kind = TYPE_KIND.get(typ, KIND_RAW)
                voxel_eff = voxel
                if kind == KIND_CLOUD and voxel_eff <= 0:
                    voxel_eff = 0.05
                cached = SOURCE.topic_data(stream)
                timeout = 60.0
                if cached is not None and now() - cached["recv"] < 3.0 and not refresh:
                    data = cached
                else:
                    client.send(
                        encode_status(
                            stream,
                            True,
                            "waiting for %s (%s)" % (stream, typ or "unknown"),
                        )
                    )
                    data = SOURCE.request_topic(stream, timeout)
                if data is None:
                    client.send(encode_error("no message on %s" % stream))
                    return
                client.send(encode_status(stream, True, "encoding"))
                frame = encode_topic_frame(stream, data, {"voxel": voxel_eff, "max": max_points or 0})
                if frame is None:
                    client.send(encode_error("unsupported message on %s" % stream))
                    return
                client.send_topic(stream, frame)
            except Exception as e:
                client.send(encode_error("%s: %s" % (stream, e)))
            finally:
                lock.release()

        t = threading.Thread(target=run)
        t.daemon = True
        t.start()

    def _service(self, client, service, args_json, request_id):
        def run():
            try:
                result = SOURCE.call_service(service, args_json)
                client.send(encode_service(request_id, service, True, json.dumps(result)))
            except Exception as e:
                client.send(encode_service(request_id, service, False, str(e)))

        t = threading.Thread(target=run)
        t.daemon = True
        t.start()

    def _topics(self, client):
        def run():
            try:
                topics = SOURCE.get_topics()
                client.send(encode_topics(topics))
            except Exception as e:
                client.send(encode_error("get_topics: %s" % e))

        t = threading.Thread(target=run)
        t.daemon = True
        t.start()

    def _raw_subscribe(self, client, topic, rate):
        if topic in client.raw_subs:
            return

        def cb(t, typ, msg):
            tnow = now()
            if tnow - client.raw_last.get(t, 0.0) < 1.0 / max(0.5, rate or 2.0):
                return
            client.raw_last[t] = tnow
            try:
                from rosbridge_library.internal.message_conversion import extract_values

                text = json.dumps(extract_values(msg))
                if len(text) > MAX_RAW_JSON:
                    text = text[:MAX_RAW_JSON] + "...(truncated)"
                stamp = 0.0
                try:
                    stamp = msg.header.stamp.to_sec()
                except Exception:
                    stamp = now()
                client.send(encode_raw(t, typ, text, stamp))
                CACHE.touch(t, len(text))
            except Exception as e:
                client.send(encode_error("raw %s: %s" % (t, e)))

        client.raw_subs[topic] = SOURCE.raw_subscribe(topic, cb)
        client.send(encode_status(topic, True, "raw subscribed"))

    def on_close(self):
        self.client.closed = True
        if self.client.twist_active:
            try:
                SOURCE.set_twist(0.0, 0.0)
            except Exception:
                pass
        streams = list(self.client.subs.keys())
        self.client.close_raw()
        with CLIENTS_LOCK:
            if self.client in CLIENTS:
                CLIENTS.remove(self.client)
        for topic in streams:
            try:
                SOURCE.release_topic(topic)
            except Exception as e:
                log("release %s: %s" % (topic, e))


def publisher_loop():
    while not STOP.is_set():
        t0 = now()
        with CLIENTS_LOCK:
            clients = list(CLIENTS)
        for client in clients:
            if client.closed:
                continue
            # 内置流 (tf/status/logs) 优先：大点云编码很慢 (ARM 上单帧可到秒级)，
            # 不能让 /slam/info 排在大点云后面被饿死。
            for stream, sub in sorted(
                client.subs.items(),
                key=lambda kv: 0 if kv[0] in BUILTIN_STREAMS else 1,
            ):
                if sub.get("rate", 0.0) <= 0:
                    continue
                interval = 1.0 / sub["rate"]
                if t0 - sub["last"] < interval:
                    continue
                try:
                    frames = frames_for_stream(stream, sub, client)
                except Exception as e:
                    log("encode %s: %s" % (stream, e))
                    continue
                if frames:
                    sub["last"] = t0
                    target = stream if stream.startswith("/") else (STREAM_TOPICS.get(stream) or [stream])[0]
                    for f in frames:
                        client.send_topic(target, f)
            if t0 - client.last_send > 10.0:
                client.send(encode_status("heartbeat", True, "alive"))
            # 手动控制看门狗：超过 0.5s 没收到新速度就发 0
            if client.twist_active and t0 - client.last_twist > 0.5:
                try:
                    SOURCE.set_twist(0.0, 0.0)
                except Exception as e:
                    log("twist stop: %s" % e)
                client.twist_active = False
            # 话题速率 / 流量统计（约 2Hz）
            if t0 - client.last_stats >= 0.5:
                dt = t0 - client.last_stats
                for topic, st in list(client.sent.items()):
                    prev = client.stat_prev.get(topic)
                    rate = 0.0
                    if prev and dt > 0:
                        rate = (st["count"] - prev[0]) / dt
                    client.stat_prev[topic] = (st["count"], t0)
                    age_ms = int(max(0.0, t0 - st["last"]) * 1000)
                    client.send(
                        encode_stat(topic, st["count"], age_ms, st["size"], st["total"], rate)
                    )
                client.last_stats = t0
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
        log("DEMO mode, serving fake map/scan/tf/slam")
    else:
        try:
            import rospy
        except ImportError:
            log("rospy not found. Source /opt/ros/<distro>/setup.bash, or use --demo.")
            return 2
        rospy.init_node("debug_bridge", anonymous=False, disable_signals=True)
        SOURCE = RosSource()
        log("ROS mode, waiting for subscriptions ...")

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
