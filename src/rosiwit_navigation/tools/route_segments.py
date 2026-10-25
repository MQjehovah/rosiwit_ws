#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""打印路线段 (几何判定): 直线段 / 圆弧 / 原地转, 以及每段起止/长度/朝向变化."""
import math
import sys

rows = []
for line in open(sys.argv[1]):
    p = line.strip().split(",")
    if len(p) >= 4 and p[0] != "x":
        rows.append((float(p[0]), float(p[1]), float(p[2])))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


segs = []          # [kind, x0,y0, x1,y1, len, dtheta]
cur = None
for i in range(1, len(rows)):
    x0, y0, t0 = rows[i - 1]
    x1, y1, t1 = rows[i]
    d = math.hypot(x1 - x0, y1 - y0)
    dt = wrap(t1 - t0)
    if d < 2e-3:
        kind = "ROT"
    elif abs(dt) > 0.01:
        kind = "ARC"
    elif abs(y1 - y0) < 0.02:
        kind = "ROW"
    else:
        kind = "LINK"
    if cur is not None and cur[0] == kind and kind in ("ROW", "ARC", "LINK"):
        cur[3] = x1
        cur[4] = y1
        cur[5] += d
        cur[6] += dt
    else:
        if cur:
            segs.append(cur)
        cur = [kind, x0, y0, x1, y1, d, dt]
if cur:
    segs.append(cur)

total = 0.0
rows_n = arcs = 0
print("%-4s %-18s %-18s %8s %8s %8s" % ("kind", "from", "to", "len", "dtheta", "R"))
for s in segs:
    total += s[5]
    extra = ""
    if s[0] == "ROW":
        rows_n += 1
    if s[0] == "ARC":
        arcs += 1
        extra = "R=%.3f" % (s[5] / abs(s[6])) if abs(s[6]) > 1e-9 else ""
    print("%-4s (%6.2f,%6.2f)  (%6.2f,%6.2f) %8.2f %8.2f %8s"
          % (s[0], s[1], s[2], s[3], s[4], s[5], s[6], extra))
print("segments=%d rows=%d arcs=%d total_len=%.2f m" % (len(segs), rows_n, arcs, total))
