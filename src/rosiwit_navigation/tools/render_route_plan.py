#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""画"规划出的"覆盖路线 (不是执行轨迹): 地图 + 作业区 + 行段/圆弧/原地转点.

用法: python3 render_route_plan.py <route.csv> <out.png> [poly.txt|rect x0 y0 x1 y1]

颜色 (按几何判定, 不信 CSV 的 kind 列):
  蓝   = 直线行段
  橙   = 圆弧 (相邻点朝向在变 = 转弯)
  红圈 = 原地转点 (位置不动)
  绿   = 其它直线 (行间连接/进场)
  灰点 = 起点
"""
import math
import struct
import sys
import zlib

CSV = sys.argv[1]
OUT = sys.argv[2]
PGM = "/home/jmq/rosiwit_ws/docs/maps/house_sim/map.pgm"
RES, OX, OY = 0.05, -2.83255, -5.86221

POLY = None
if len(sys.argv) >= 5 and sys.argv[3].endswith(".txt"):
    POLY = []
    for line in open(sys.argv[3]):
        p = line.strip().split(",")
        if len(p) >= 2 and not line.startswith("#"):
            POLY.append((float(p[0]), float(p[1])))
elif len(sys.argv) >= 7:
    x0, y0, x1, y1 = (float(v) for v in sys.argv[3:7])
    POLY = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]


class Canvas(object):
    def __init__(self, w, h, bg=(255, 255, 255)):
        self.w, self.h = w, h
        self.px = bytearray()
        row = bytes(bytearray(bg)) * w
        for _ in range(h):
            self.px.extend(row)

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
            for oy in range(-(t // 2), t // 2 + 1):
                for ox in range(-(t // 2), t // 2 + 1):
                    self.set(cx + ox, cy + oy, c)

    def circle(self, cx, cy, r, c, thickness=None):
        for y in range(cy - r, cy + r + 1):
            for x in range(cx - r, cx + r + 1):
                d = math.hypot(x - cx, y - cy)
                if thickness is None:
                    if d <= r:
                        self.set(x, y, c)
                elif abs(d - r) <= thickness + 0.5:
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
        open(path, "wb").write(png)


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def load_pgm(path):
    data = open(path, "rb").read()
    parts, idx = [], 0
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
    rows = []
    for line in open(CSV):
        p = line.strip().split(",")
        if len(p) >= 4 and p[0] != "x":
            try:
                rows.append((float(p[0]), float(p[1]), float(p[2])))
            except ValueError:
                pass
    if not rows:
        print("empty route")
        return 1

    w, h, img = load_pgm(PGM)
    # 只画路线附近的区域
    xs = [r[0] for r in rows]
    ys = [r[1] for r in rows]
    m = 1.2
    vx0, vx1 = min(xs) - m, max(xs) + m
    vy0, vy1 = min(ys) - m, max(ys) + m
    if POLY:
        vx0 = min(vx0, min(p[0] for p in POLY) - 0.6)
        vx1 = max(vx1, max(p[0] for p in POLY) + 0.6)
        vy0 = min(vy0, min(p[1] for p in POLY) - 0.6)
        vy1 = max(vy1, max(p[1] for p in POLY) + 0.6)

    scale = 16
    cw = int(round((vx1 - vx0) / RES * scale))
    ch = int(round((vy1 - vy0) / RES * scale))
    canvas = Canvas(cw, ch)

    def to_px(wx, wy):
        return (int(round((wx - vx0) / RES * scale)),
                int(round((vy1 - wy) / RES * scale)))

    gx0 = int((vx0 - OX) / RES)
    gx1 = int((vx1 - OX) / RES)
    gy0 = int((vy0 - OY) / RES)
    gy1 = int((vy1 - OY) / RES)
    for gy in range(gy0, gy1):
        for gx in range(gx0, gx1):
            if not (0 <= gx < w and 0 <= gy < h):
                c = (255, 255, 255)
            else:
                v = img[(h - 1 - gy) * w + gx]
                c = (60, 60, 60) if v < 50 else ((248, 248, 248) if v > 250 else (225, 225, 225))
            px, py = to_px(OX + (gx + 0.5) * RES, OY + (gy + 0.5) * RES)
            canvas.rect(px - scale // 2, py - scale // 2, scale, scale, c)

    if POLY:
        pts = [to_px(x, y) for (x, y) in POLY]
        for i in range(len(pts)):
            a, b = pts[i], pts[(i + 1) % len(pts)]
            canvas.line(a[0], a[1], b[0], b[1], (200, 0, 180), 3)

    ROW_C, ARC_C, LINK_C, ROT_C = (30, 80, 220), (245, 130, 20), (20, 150, 80), (220, 30, 30)
    arrows = 0
    first_straight = True
    for i in range(1, len(rows)):
        x0, y0, t0 = rows[i - 1]
        x1, y1, t1 = rows[i]
        p0, p1 = to_px(x0, y0), to_px(x1, y1)
        d = math.hypot(x1 - x0, y1 - y0)
        dt = abs(wrap(t1 - t0))
        if d < 2e-3:
            canvas.circle(p1[0], p1[1], 7, ROT_C, thickness=2)
            continue
        if dt > 0.01:
            color = ARC_C
        elif first_straight:
            color = LINK_C      # 进场直线 (从起点到第一行行首)
            first_straight = False
        else:
            color = ROW_C
        canvas.line(p0[0], p0[1], p1[0], p1[1], color, 3)
        # 每 ~0.6 m 一个方向箭头
        arrows += 1
        if arrows % 30 == 0:
            canvas.circle(p1[0], p1[1], 3, (120, 20, 20))

    p = to_px(rows[0][0], rows[0][1])
    canvas.circle(p[0], p[1], 9, (110, 110, 110))
    canvas.png(OUT)
    print("wrote %s (%dx%d, %d pts)" % (OUT, cw, ch, len(rows)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
