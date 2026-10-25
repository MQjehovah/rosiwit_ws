#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 route_test2.py 的轨迹画到地图上 (纯 python, 不依赖 numpy/PIL).

用法:
    python3 render_route.py [csv] [pgm] [out.png]

默认:
    csv  = /tmp/route_poses.csv   (route_test2.py 输出: t,x,y,yaw)
    pgm  = docs/maps/house_sim/map.pgm
    out  = /tmp/route.png

颜色: 地图 白=自由/深灰=障碍/浅灰=未知; 轨迹 蓝(慢)->红(快); 目标 绿圈;
      最小余量位置 橙色圈.
"""
import math
import os
import struct
import sys
import zlib

CSV = sys.argv[1] if len(sys.argv) > 1 else "/tmp/route_poses.csv"
PGM = sys.argv[2] if len(sys.argv) > 2 else "/home/jmq/rosiwit_ws/docs/maps/house_sim/map.pgm"
OUT = sys.argv[3] if len(sys.argv) > 3 else "/tmp/route.png"

RES, OX, OY = 0.05, -2.83255, -5.86221
GOALS = [(1.5, 0.0), (3.0, -2.0), (-1.5, 0.0)]
CLEARANCE_POSE = None   # 由调用方写 /tmp/route_clearance.txt: x,y


class Canvas(object):
    def __init__(self, w, h, bg=(255, 255, 255)):
        self.w = w
        self.h = h
        self.px = bytearray()
        for _ in range(w * h):
            self.px.extend(bg)

    def set(self, x, y, c):
        if 0 <= x < self.w and 0 <= y < self.h:
            i = (y * self.w + x) * 3
            self.px[i:i + 3] = bytes(bytearray(c))

    def rect(self, x0, y0, w, h, c):
        for yy in range(y0, y0 + h):
            for xx in range(x0, x0 + w):
                self.set(xx, yy, c)

    def line(self, x0, y0, x1, y1, c, t=1):
        n = int(max(abs(x1 - x0), abs(y1 - y0))) + 1
        for i in range(n + 1):
            f = float(i) / n
            cx = int(round(x0 + (x1 - x0) * f))
            cy = int(round(y0 + (y1 - y0) * f))
            for oy in range(-t // 2, t // 2 + 1):
                for ox in range(-t // 2, t // 2 + 1):
                    self.set(cx + ox, cy + oy, c)

    def circle(self, cx, cy, r, c, thickness=None):
        for y in range(cy - r, cy + r + 1):
            for x in range(cx - r, cx + r + 1):
                d = math.hypot(x - cx, y - cy)
                if thickness is None:
                    if d <= r:
                        self.set(x, y, c)
                elif abs(d - r) <= thickness:
                    self.set(x, y, c)

    def png(self, path):
        raw = bytearray()
        stride = self.w * 3
        for y in range(self.h):
            raw.append(0)
            raw.extend(self.px[y * stride:(y + 1) * stride])

        def chunk(tag, data):
            return (struct.pack(">I", len(data)) + tag + data +
                    struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF))

        png = b"\x89PNG\r\n\x1a\n"
        png += chunk(b"IHDR", struct.pack(">IIBBBBB", self.w, self.h, 8, 2, 0, 0, 0))
        png += chunk(b"IDAT", zlib.compress(bytes(raw), 9))
        png += chunk(b"IEND", b"")
        with open(path, "wb") as f:
            f.write(png)


def load_pgm(path):
    data = open(path, "rb").read()
    parts = []
    idx = 0
    while len(parts) < 4:
        while data[idx:idx + 1].isspace():
            idx += 1
        if data[idx:idx + 1] == b"#":
            while data[idx:idx + 1] not in (b"\n", b""):
                idx += 1
            continue
        s = idx
        while not data[idx:idx + 1].isspace():
            idx += 1
        parts.append(data[s:idx])
    idx += 1
    w, h = int(parts[1]), int(parts[2])
    return w, h, data[idx:idx + w * h]


def main():
    if not os.path.exists(CSV):
        print("missing %s (run route_test2.py first)" % CSV)
        return 1
    rows = []
    for line in open(CSV):
        p = line.strip().split(",")
        if len(p) >= 4:
            try:
                rows.append(tuple(float(v) for v in p))
            except ValueError:
                pass
    if not rows:
        print("empty trajectory")
        return 1

    w, h, img = load_pgm(PGM)
    scale = 4
    margin = 12
    cw = w * scale + 2 * margin
    ch = h * scale + 2 * margin
    canvas = Canvas(cw, ch)

    def to_px(wx, wy):
        gx = (wx - OX) / RES
        gy = (wy - OY) / RES
        return (margin + int(round(gx * scale)),
                margin + int(round((h - 1 - gy) * scale)))

    for gy in range(h):
        for gx in range(w):
            v = img[(h - 1 - gy) * w + gx]   # PGM 首行是图像顶部
            if v < 50:
                c = (70, 70, 70)
            elif v > 250:
                c = (245, 245, 245)
            else:
                c = (205, 205, 205)
            px, py = to_px(OX + (gx + 0.5) * RES, OY + (gy + 0.5) * RES)
            canvas.rect(px - scale // 2, py - scale // 2, scale, scale, c)

    # 轨迹按速度着色
    for i in range(1, len(rows)):
        (t0, x0, y0, _) = rows[i - 1]
        (t1, x1, y1, _) = rows[i]
        dt = max(1e-3, t1 - t0)
        v = math.hypot(x1 - x0, y1 - y0) / dt
        k = max(0.0, min(1.0, v / 0.7))
        color = (int(40 + 200 * k), int(90 - 60 * k), int(230 - 200 * k))
        p0 = to_px(x0, y0)
        p1 = to_px(x1, y1)
        if math.hypot(p1[0] - p0[0], p1[1] - p0[1]) < 60:
            canvas.line(p0[0], p0[1], p1[0], p1[1], color, 2)

    for (gx, gy) in GOALS:
        p = to_px(gx, gy)
        canvas.circle(p[0], p[1], 9, (0, 150, 0), thickness=2)
    p = to_px(rows[0][1], rows[0][2])
    canvas.circle(p[0], p[1], 5, (120, 120, 120))
    p = to_px(rows[-1][1], rows[-1][2])
    canvas.circle(p[0], p[1], 6, (220, 30, 30))

    if os.path.exists("/tmp/route_clearance.txt"):
        try:
            cx, cy = [float(v) for v in open("/tmp/route_clearance.txt").read().split(",")[:2]]
            p = to_px(cx, cy)
            canvas.circle(p[0], p[1], 11, (255, 140, 0), thickness=2)
        except Exception:  # noqa: BLE001
            pass

    canvas.png(OUT)
    print("wrote %s (%dx%d, %d samples)" % (OUT, cw, ch, len(rows)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
