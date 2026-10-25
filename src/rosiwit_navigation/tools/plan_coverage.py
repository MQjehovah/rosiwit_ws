#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""离线评估"规划出来的路线"能覆盖多少面积 (不跑仿真).

覆盖模型与仿真里一致: 车心沿路走, 半径 sweep/2 的圆盘扫过算已覆盖;
只统计多边形内、且不是静态障碍的格子.

用法: python3 plan_coverage.py <route.csv> <poly.txt> [sweep=0.5] [half_width=0.31] [out.png]
"""
import math
import struct
import sys
import zlib

CSV, POLYF = sys.argv[1], sys.argv[2]
SWEEP = float(sys.argv[3]) if len(sys.argv) > 3 else 0.5
HALF = float(sys.argv[4]) if len(sys.argv) > 4 else 0.31
OUT = sys.argv[5] if len(sys.argv) > 5 else None
PGM = "/home/jmq/rosiwit_ws/docs/maps/house_sim/map.pgm"
RES, OX, OY = 0.05, -2.83255, -5.86221


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


def point_in_poly(px, py, poly):
    inside = False
    n = len(poly)
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if (yi > py) != (yj > py):
            xint = xi + (py - yi) * (xj - xi) / (yj - yi)
            if px < xint:
                inside = not inside
        j = i
    return inside


w, h, img = load_pgm(PGM)
poly = []
for line in open(POLYF):
    p = line.strip().split(",")
    if len(p) >= 2 and not line.startswith("#"):
        poly.append((float(p[0]), float(p[1])))
xs = [p[0] for p in poly]
ys = [p[1] for p in poly]
cx0, cx1 = min(xs), max(xs)
cy0, cy1 = min(ys), max(ys)
gx0 = max(0, int((cx0 - OX) / RES) - 2)
gx1 = min(w, int((cx1 - OX) / RES) + 2)
gy0 = max(0, int((cy0 - OY) / RES) - 2)
gy1 = min(h, int((cy1 - OY) / RES) + 2)

# 区域格子 (多边形内、非障碍)
area_cells = {}
for gy in range(gy0, gy1):
    for gx in range(gx0, gx1):
        v = img[(h - 1 - gy) * w + gx]
        if v < 50:
            continue
        x = OX + (gx + 0.5) * RES
        y = OY + (gy + 0.5) * RES
        if point_in_poly(x, y, poly):
            area_cells[(gx, gy)] = 0

route = []
for line in open(CSV):
    p = line.strip().split(",")
    if len(p) >= 4 and p[0] != "x":
        route.append((float(p[0]), float(p[1])))

r = SWEEP / 2.0
r_cells = int(math.ceil(r / RES))
covered = set()
for (x, y) in route:
    gx = int((x - OX) / RES)
    gy = int((y - OY) / RES)
    for dy in range(-r_cells, r_cells + 1):
        for dx in range(-r_cells, r_cells + 1):
            c = (gx + dx, gy + dy)
            if c not in area_cells:
                continue
            wx = OX + (c[0] + 0.5) * RES
            wy = OY + (c[1] + 0.5) * RES
            if math.hypot(wx - x, wy - y) <= r + 0.5 * RES:
                covered.add(c)

total = len(area_cells)
cov = len(covered)
cell_area = RES * RES
path_len = sum(math.hypot(route[i][0] - route[i - 1][0], route[i][1] - route[i - 1][1])
               for i in range(1, len(route)))
print("region=%.2f m2  covered=%.2f m2  rate=%.1f%%  path=%.2f m  (pts=%d)"
      % (total * cell_area, cov * cell_area,
         100.0 * cov / max(1, total), path_len, len(route)))

if OUT:
    scale = 12
    cw = int((cx1 - cx0 + 0.4) / RES * scale)
    ch = int((cy1 - cy0 + 0.4) / RES * scale)
    px = bytearray()
    for yy in range(ch):
        px.extend(bytes(bytearray((255, 255, 255))) * cw)

    def put(x, y, c):
        if 0 <= x < cw and 0 <= y < ch:
            i = (y * cw + x) * 3
            px[i:i + 3] = bytes(bytearray(c))

    def to_px(wx, wy):
        return (int((wx - cx0 + 0.2) / RES * scale),
                int((cy1 + 0.2 - wy) / RES * scale))

    for (gx, gy) in area_cells:
        wx = OX + (gx + 0.5) * RES
        wy = OY + (gy + 0.5) * RES
        c = (150, 220, 150) if (gx, gy) in covered else (230, 90, 90)
        ax, ay = to_px(wx, wy)
        for oy in range(scale):
            for ox in range(scale):
                put(ax + ox - scale // 2, ay + oy - scale // 2, c)
    for i in range(1, len(route)):
        a = to_px(*route[i - 1])
        b = to_px(*route[i])
        n = int(max(abs(b[0] - a[0]), abs(b[1] - a[1]))) + 1
        for k in range(n + 1):
            f = float(k) / n
            put(int(a[0] + (b[0] - a[0]) * f), int(a[1] + (b[1] - a[1]) * f), (20, 20, 90))

    raw = bytearray()
    for yy in range(ch):
        raw.append(0)
        raw.extend(px[yy * cw * 3:(yy + 1) * cw * 3])

    def chunk(tag, data):
        return (struct.pack(">I", len(data)) + tag + data +
                struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF))

    png = b"\x89PNG\r\n\x1a\n"
    png += chunk(b"IHDR", struct.pack(">IIBBBBB", cw, ch, 8, 2, 0, 0, 0))
    png += chunk(b"IDAT", zlib.compress(bytes(raw), 9))
    png += chunk(b"IEND", b"")
    open(OUT, "wb").write(png)
    print("wrote %s" % OUT)
