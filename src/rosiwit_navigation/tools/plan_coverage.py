#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""离线评估规划路线的覆盖率 (不跑仿真).

两个分母:
  region    : 多边形内、非障碍 (与仿真统计的 total 对应)
  coverable : 车心合法位置 (离障碍 >= robot_inset) 外扩 sweep/2
              = 物理上能被扫到的面积. 家具口袋/窄缝不算.

用法: python3 plan_coverage.py <route.csv> <poly.txt> [sweep=0.5] [robot_inset=0.31] [out.png]
"""
import math
import struct
import sys
import zlib

CSV, POLYF = sys.argv[1], sys.argv[2]
SWEEP = float(sys.argv[3]) if len(sys.argv) > 3 else 0.5
INSET = float(sys.argv[4]) if len(sys.argv) > 4 else 0.31
OUT = sys.argv[5] if len(sys.argv) > 5 else None
PGM = "/home/jmq/rosiwit_ws/docs/maps/house_sim/map.pgm"
RES, OX, OY = 0.05, -2.83255, -5.86221
INF = 1e6


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


def chamfer(src, w, h):
    d = [[0.0 if src[y][x] else INF for x in range(w)] for y in range(h)]
    for y in range(h):
        for x in range(w):
            v = d[y][x]
            if v <= 0.0:
                continue
            best = v
            if x > 0:
                best = min(best, d[y][x - 1] + 1.0)
            if y > 0:
                best = min(best, d[y - 1][x] + 1.0)
                if x > 0:
                    best = min(best, d[y - 1][x - 1] + 1.414)
                if x + 1 < w:
                    best = min(best, d[y - 1][x + 1] + 1.414)
            d[y][x] = best
    for y in range(h - 1, -1, -1):
        for x in range(w - 1, -1, -1):
            v = d[y][x]
            if v <= 0.0:
                continue
            best = v
            if x + 1 < w:
                best = min(best, d[y][x + 1] + 1.0)
            if y + 1 < h:
                best = min(best, d[y + 1][x] + 1.0)
                if x + 1 < w:
                    best = min(best, d[y + 1][x + 1] + 1.414)
                if x > 0:
                    best = min(best, d[y + 1][x - 1] + 1.414)
            d[y][x] = best
    return d


mw, mh, img = load_pgm(PGM)
poly = []
for line in open(POLYF):
    p = line.strip().split(",")
    if len(p) >= 2 and not line.startswith("#"):
        poly.append((float(p[0]), float(p[1])))
cxs = [p[0] for p in poly]
cys = [p[1] for p in poly]
cx0, cx1 = min(cxs), max(cxs)
cy0, cy1 = min(cys), max(cys)
gx0 = max(0, int((cx0 - OX) / RES) - 3)
gx1 = min(mw, int((cx1 - OX) / RES) + 3)
gy0 = max(0, int((cy0 - OY) / RES) - 3)
gy1 = min(mh, int((cy1 - OY) / RES) + 3)
W = gx1 - gx0
H = gy1 - gy0

inside = [[False] * W for _ in range(H)]
obs = [[False] * W for _ in range(H)]
for y in range(H):
    for x in range(W):
        v = img[(mh - 1 - (gy0 + y)) * mw + (gx0 + x)]
        obs[y][x] = (v < 50)
        if not obs[y][x]:
            wx = OX + (gx0 + x + 0.5) * RES
            wy = OY + (gy0 + y + 0.5) * RES
            inside[y][x] = point_in_poly(wx, wy, poly)

d_obs = chamfer(obs, W, H)          # 到障碍的距离 (格)

region = 0
center_ok = [[False] * W for _ in range(H)]
for y in range(H):
    for x in range(W):
        if not inside[y][x]:
            continue
        region += 1
        if d_obs[y][x] * RES >= INSET:
            center_ok[y][x] = True

# coverable = center_ok 外扩 sweep/2
r_cells = int(math.ceil((SWEEP / 2.0) / RES))
coverable = [[False] * W for _ in range(H)]
for y in range(H):
    for x in range(W):
        if not center_ok[y][x]:
            continue
        for dy in range(-r_cells, r_cells + 1):
            ny = y + dy
            if ny < 0 or ny >= H:
                continue
            for dx in range(-r_cells, r_cells + 1):
                nx = x + dx
                if nx < 0 or nx >= W:
                    continue
                if dx * dx + dy * dy > r_cells * r_cells:
                    continue
                if not inside[ny][nx]:
                    continue
                coverable[ny][nx] = True

route = []
for line in open(CSV):
    p = line.strip().split(",")
    if len(p) >= 4 and p[0] != "x":
        route.append((float(p[0]), float(p[1])))

covered = [[False] * W for _ in range(H)]
r = SWEEP / 2.0 + 0.5 * RES
for (x, y) in route:
    gx = int((x - OX) / RES) - gx0
    gy = int((y - OY) / RES) - gy0
    for dy in range(-r_cells, r_cells + 1):
        ny = gy + dy
        if ny < 0 or ny >= H:
            continue
        for dx in range(-r_cells, r_cells + 1):
            nx = gx + dx
            if nx < 0 or nx >= W:
                continue
            if math.hypot(dx * RES, dy * RES) <= r:
                covered[ny][nx] = True

area = RES * RES
n_region = region
n_coverable = sum(1 for y in range(H) for x in range(W) if coverable[y][x])
n_cov = sum(1 for y in range(H) for x in range(W)
            if coverable[y][x] and covered[y][x])
n_cov_region = sum(1 for y in range(H) for x in range(W)
                   if inside[y][x] and covered[y][x])
path_len = sum(math.hypot(route[i][0] - route[i - 1][0], route[i][1] - route[i - 1][1])
               for i in range(1, len(route)))
print("region=%.2f m2  coverable=%.2f m2  covered=%.2f m2  "
      "rate=%.1f%% (of coverable)  %.1f%% (of region)  path=%.2f m"
      % (n_region * area, n_coverable * area, n_cov * area,
         100.0 * n_cov / max(1, n_coverable),
         100.0 * n_cov_region / max(1, n_region), path_len))

if OUT:
    scale = 12
    cw = int((cx1 - cx0 + 0.4) / RES * scale)
    ch = int((cy1 - cy0 + 0.4) / RES * scale)
    px = bytearray(bytes(bytearray((255, 255, 255))) * (cw * ch))

    def put(x, y, c):
        if 0 <= x < cw and 0 <= y < ch:
            i = (y * cw + x) * 3
            px[i:i + 3] = bytes(bytearray(c))

    def to_px(wx, wy):
        return (int((wx - cx0 + 0.2) / RES * scale),
                int((cy1 + 0.2 - wy) / RES * scale))

    for y in range(H):
        for x in range(W):
            if not (inside[y][x] or coverable[y][x]):
                continue
            wx = OX + (gx0 + x + 0.5) * RES
            wy = OY + (gy0 + y + 0.5) * RES
            if coverable[y][x]:
                c = (150, 215, 150) if covered[y][x] else (230, 80, 80)
            else:
                c = (200, 200, 200)
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
