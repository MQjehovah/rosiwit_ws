#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 coverage_check.py 落盘的数据画成 PNG (纯 python, 不依赖 numpy/PIL).

用法:
    python3 render_coverage.py [数据目录] [输出png]

颜色:
    静态地图: 自由=白, 障碍=深灰, 未知=浅灰
    覆盖状态覆盖上去: 未覆盖=红(半透明), 已覆盖=绿, 阻挡=橙, 障碍=深灰
    轨迹: 蓝色折线 + 终点红点
    区域多边形: 品红虚线
"""
import json
import math
import os
import struct
import sys
import zlib

DATA = sys.argv[1] if len(sys.argv) > 2 else "/tmp/cov"
OUT = sys.argv[2] if len(sys.argv) > 2 else os.path.join(DATA, "coverage.png")

SCALE = 4        # 每栅格放大倍数
MARGIN = 12      # 边距 (像素)


def read_meta():
    with open(os.path.join(DATA, "meta.json")) as f:
        return json.load(f)


def read_bytes(name, n):
    path = os.path.join(DATA, name)
    if not os.path.exists(path):
        return None
    with open(path, "rb") as f:
        raw = f.read()
    return [b - 256 if b >= 128 else b for b in raw[:n]]


def read_csv(name):
    rows = []
    path = os.path.join(DATA, name)
    if not os.path.exists(path):
        return rows
    with open(path) as f:
        for line in f:
            parts = line.strip().split(",")
            if len(parts) >= 3:
                try:
                    rows.append(tuple(float(p) for p in parts))
                except ValueError:
                    pass
    return rows


class Canvas(object):
    def __init__(self, w, h, bg=(255, 255, 255)):
        self.w = w
        self.h = h
        self.px = bytearray()
        for _ in range(w * h):
            self.px.extend(bg)

    def set(self, x, y, color):
        if 0 <= x < self.w and 0 <= y < self.h:
            i = (y * self.w + x) * 3
            self.px[i:i + 3] = bytes(bytearray(color))

    def rect(self, x0, y0, w, h, color):
        for yy in range(y0, y0 + h):
            for xx in range(x0, x0 + w):
                self.set(xx, yy, color)

    def line(self, x0, y0, x1, y1, color, thickness=1):
        dx = x1 - x0
        dy = y1 - y0
        steps = int(max(abs(dx), abs(dy))) + 1
        for i in range(steps + 1):
            t = float(i) / max(1, steps)
            cx = int(round(x0 + dx * t))
            cy = int(round(y0 + dy * t))
            for oy in range(-thickness // 2, thickness // 2 + 1):
                for ox in range(-thickness // 2, thickness // 2 + 1):
                    self.set(cx + ox, cy + oy, color)

    def circle(self, cx, cy, r, color):
        rr = r * r
        for y in range(cy - r, cy + r + 1):
            for x in range(cx - r, cx + r + 1):
                if (x - cx) ** 2 + (y - cy) ** 2 <= rr:
                    self.set(x, y, color)

    def png(self, path):
        raw = bytearray()
        stride = self.w * 3
        for y in range(self.h):
            raw.append(0)   # filter: none
            raw.extend(self.px[y * stride:(y + 1) * stride])

        def chunk(tag, data):
            out = struct.pack(">I", len(data)) + tag + data
            return out + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)

        png = b"\x89PNG\r\n\x1a\n"
        png += chunk(b"IHDR", struct.pack(">IIBBBBB", self.w, self.h, 8, 2, 0, 0, 0))
        png += chunk(b"IDAT", zlib.compress(bytes(raw), 9))
        png += chunk(b"IEND", b"")
        with open(path, "wb") as f:
            f.write(png)


def main():
    meta = read_meta()
    w = meta["width"]
    h = meta["height"]
    if w == 0 or h == 0:
        print("empty meta")
        return 1
    world = read_bytes("map.bin", w * h)
    cover = read_bytes("clean.bin", w * h)
    traj = read_csv("traj.csv")
    poly = meta.get("polygon") or []

    scale = SCALE
    cw = w * scale + 2 * MARGIN
    ch = h * scale + 2 * MARGIN
    canvas = Canvas(cw, ch)

    def to_px(wx, wy):
        """世界坐标 -> 像素 (图像 y 翻转)."""
        gx = (wx - meta["origin_x"]) / meta["resolution"]
        gy = (wy - meta["origin_y"]) / meta["resolution"]
        return (MARGIN + int(round(gx * scale)),
                MARGIN + int(round((h - 1 - gy) * scale)))

    # 底图 + 覆盖
    for gy in range(h):
        for gx in range(w):
            idx = gy * w + gx
            base = (245, 245, 245)
            v = world[idx] if world else 0
            if world is not None:
                if v >= 100:
                    base = (70, 70, 70)
                elif v < 0:
                    base = (205, 205, 205)
            if cover is not None:
                c = cover[idx]
                if c == 0:
                    base = (250, 120, 120)        # 未覆盖
                elif c == 50:
                    base = (90, 190, 110)         # 已覆盖
                elif c == 80:
                    base = (245, 170, 60)         # 阻挡
                elif c == 100:
                    base = (70, 70, 70)           # 障碍
            px, py = to_px(meta["origin_x"] + (gx + 0.5) * meta["resolution"],
                           meta["origin_y"] + (gy + 0.5) * meta["resolution"])
            canvas.rect(px - scale // 2, py - scale // 2, scale, scale, base)

    # 多边形
    if len(poly) >= 3:
        pts = [to_px(x, y) for (x, y) in poly]
        for i in range(len(pts)):
            a = pts[i]
            b = pts[(i + 1) % len(pts)]
            canvas.line(a[0], a[1], b[0], b[1], (200, 0, 180), 2)

    # 轨迹
    prev = None
    for (t, x, y, yaw) in traj:
        p = to_px(x, y)
        if prev is not None and math.hypot(p[0] - prev[0], p[1] - prev[1]) < 60:
            canvas.line(prev[0], prev[1], p[0], p[1], (30, 80, 220), 2)
        prev = p
    if traj:
        p = to_px(traj[-1][1], traj[-1][2])
        canvas.circle(p[0], p[1], 5, (220, 30, 30))

    canvas.png(OUT)
    print("wrote %s  (%dx%d, %.1f%% covered, %.2f m2)"
          % (OUT, cw, ch, meta["coverage_rate"] * 100.0, meta["clean_area"]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
