#!/usr/bin/env python3
"""从全局 PCD 离线生成 2D 占据栅格 (PGM/YAML).

- 局部地面: 5m 分块, 每块取 z 的 2% 分位作为地面 -> 跟随本包 30-55m 的 z 漂移
- 障碍: 相对地面 [0.1, 2.0]m 的点
- 墙体后处理: close 3 (桥接稀疏点) -> 去小连通域 (<80 格) -> dilate 1
- 自由空间: 从轨迹种子在"墙以外 + 距轨迹 8m 内"做连通域填充

输出约定与 rosiwit_slam/grid_map_io.cpp 一致:
  P5, 首行 = 最大 y, 0=occupied, 254=free, 128=unknown.
"""
import os, sys, math
import numpy as np
import cv2
from PIL import Image, ImageDraw

RUN = sys.argv[1] if len(sys.argv) > 1 else "/home/jmq/runs/rosiwit_genz_full_noclamp"
OUT = sys.argv[2] if len(sys.argv) > 2 else RUN + "/map.pgm"
RES = float(sys.argv[3]) if len(sys.argv) > 3 else 0.05
# 导出参数可用环境变量覆盖, 便于离线反复试参 (不需要重跑建图)
BLOCK = float(os.environ.get("PGM_BLOCK", "5.0"))
MIN_H = float(os.environ.get("PGM_MIN_H", "0.1"))
MAX_H = float(os.environ.get("PGM_MAX_H", "2.0"))
FREE_R = float(os.environ.get("PGM_FREE_R", "8.0"))
CLOSE = int(os.environ.get("PGM_CLOSE", "1"))
DENOISE_MIN = int(os.environ.get("PGM_DENOISE", "0"))
DILATE = int(os.environ.get("PGM_DILATE", "1"))

def read_pcd(path):
    with open(path, "rb") as f:
        fields = sizes = None; n = 0
        while True:
            line = f.readline().decode("ascii", "ignore").strip()
            if not line: continue
            k, _, v = line.partition(" ")
            if k == "FIELDS": fields = v.split()
            elif k == "SIZE": sizes = [int(x) for x in v.split()]
            elif k == "POINTS": n = int(v)
            elif k == "DATA": break
        raw = f.read()
    offs, off = [], 0
    for s in sizes: offs.append(off); off += s
    dt = np.dtype({"names": fields, "formats": ["<f4"]*len(fields),
                   "offsets": offs, "itemsize": off})
    a = np.frombuffer(raw, dtype=dt, count=n)
    return np.stack([a["x"], a["y"], a["z"]], 1).astype(np.float64)

def local_ground(x, y, z, x0, y0):
    nby = int(math.floor((y.max() - y0) / BLOCK)) + 1
    bx = np.floor((x - x0) / BLOCK).astype(np.int64)
    by = np.floor((y - y0) / BLOCK).astype(np.int64)
    bid = bx * nby + by
    order = np.argsort(bid)
    bs = bid[order]
    starts = np.r_[0, np.where(np.diff(bs) != 0)[0] + 1]
    ends = np.r_[starts[1:], len(bs)]
    g = np.empty(len(x), dtype=np.float64)
    for s, e in zip(starts, ends):
        idx = order[s:e]
        g[idx] = np.percentile(z[idx], 2.0)
    return g

def main():
    print("loading", RUN + "/map.pcd", flush=True)
    P = read_pcd(RUN + "/map.pcd")
    x, y, z = P[:, 0], P[:, 1], P[:, 2]
    x0 = np.percentile(x, 0.02) - 1.0
    x1 = np.percentile(x, 99.98) + 1.0
    y0 = np.percentile(y, 0.02) - 1.0
    y1 = np.percentile(y, 99.98) + 1.0
    W = int(math.ceil((x1 - x0) / RES)) + 1
    H = int(math.ceil((y1 - y0) / RES)) + 1
    print("points %d  grid %dx%d  res %.3f  extent %.0fx%.0f" % (
        len(P), W, H, RES, x1 - x0, y1 - y0), flush=True)

    print("local ground (block %.1fm)..." % BLOCK, flush=True)
    g = local_ground(x, y, z, x0, y0)
    rel = z - g
    print("relative height p1/p50/p99 = %.2f / %.2f / %.2f" % tuple(
        np.percentile(rel, [1, 50, 99])), flush=True)

    ix = np.clip(((x - x0) / RES).astype(np.int64), 0, W - 1)
    iy = np.clip(((y - y0) / RES).astype(np.int64), 0, H - 1)
    cid = iy * W + ix
    band = (rel >= MIN_H) & (rel <= MAX_H)
    occ = np.zeros(W * H, dtype=np.uint8)
    occ[np.unique(cid[band])] = 1
    occ = occ.reshape(H, W)
    print("occupied raw %d cells (%.2f%%)" % (occ.sum(), 100.0 * occ.mean()), flush=True)

    # 墙体后处理: close -> denoise -> dilate
    kc = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * CLOSE + 1, 2 * CLOSE + 1))
    occ_u = cv2.morphologyEx(occ, cv2.MORPH_CLOSE, kc)
    num, labels, stats, _ = cv2.connectedComponentsWithStats(occ_u, connectivity=8)
    keep = np.zeros(num, dtype=bool)
    keep[0] = False
    for i in range(1, num):
        keep[i] = stats[i, cv2.CC_STAT_AREA] >= DENOISE_MIN
    occ_u = keep[labels].astype(np.uint8)
    if DILATE > 0:
        kd = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * DILATE + 1, 2 * DILATE + 1))
        occ_u = cv2.dilate(occ_u, kd)
    print("occupied after close%d/denoise%d/dilate%d: %d cells (%.2f%%), comps kept %d" % (
        CLOSE, DENOISE_MIN, DILATE, occ_u.sum(), 100.0 * occ_u.mean(), int(keep.sum())), flush=True)

    d = np.genfromtxt(RUN + "/odom_q.csv", delimiter=",", names=True)
    tx = np.clip(((d["x"] - x0) / RES).astype(np.int64), 0, W - 1)
    ty = np.clip(((d["y"] - y0) / RES).astype(np.int64), 0, H - 1)
    traj = np.zeros((H, W), dtype=np.uint8)
    traj[ty, tx] = 1

    r = max(1, int(FREE_R / RES))
    ksq = cv2.getStructuringElement(cv2.MORPH_RECT, (2 * r + 1, 2 * r + 1))
    near = cv2.dilate(traj, ksq) > 0
    passable = (occ_u == 0)
    region = (passable & near).astype(np.uint8)
    n2, lab2 = cv2.connectedComponents(region, connectivity=4)
    seeds = np.unique(lab2[traj > 0])
    keep2 = np.zeros(n2, dtype=bool)
    keep2[seeds] = True
    keep2[0] = False
    free = keep2[lab2] & passable
    print("free components %d, kept %d; free %.2f%%" % (n2, int(keep2.sum()), 100.0 * free.mean()), flush=True)

    img = np.full((H, W), 128, dtype=np.uint8)
    img[free] = 254
    img[occ_u > 0] = 0
    print("final occupied %.2f%%  free %.2f%%  unknown %.2f%%" % (
        100.0 * (img == 0).mean(), 100.0 * (img == 254).mean(), 100.0 * (img == 128).mean()))

    pgm = OUT
    os.makedirs(os.path.dirname(pgm), exist_ok=True)
    with open(pgm, "wb") as f:
        f.write(b"P5\n%d %d\n255\n" % (W, H))
        f.write(np.ascontiguousarray(img[::-1]).tobytes())
    yml = os.path.splitext(pgm)[0] + ".yaml"
    with open(yml, "w") as f:
        f.write("image: %s\n" % os.path.basename(pgm))
        f.write("resolution: %.6f\n" % RES)
        f.write("origin: [%.4f, %.4f, 0.0]\n" % (x0, y0))
        f.write("negate: 0\noccupied_thresh: 0.59\nfree_thresh: 0.49\nmode: trinary\n")
    print("saved", pgm, yml, flush=True)

    up = img[::-1]
    rgb = np.empty((H, W, 3), dtype=np.uint8)
    rgb[up == 254] = (255, 255, 255)
    rgb[up == 128] = (110, 110, 110)
    rgb[up == 0] = (0, 0, 0)
    im = Image.fromarray(rgb)
    dr = ImageDraw.Draw(im)
    pts = [(int(a), int(H - 1 - b)) for a, b in zip(tx, ty)]
    for a, b in zip(pts[:-1], pts[1:]):
        dr.line([a, b], fill=(255, 0, 0), width=3)
    dr.ellipse([pts[0][0] - 6, pts[0][1] - 6, pts[0][0] + 6, pts[0][1] + 6], outline=(0, 255, 0), width=3)
    dr.ellipse([pts[-1][0] - 6, pts[-1][1] - 6, pts[-1][0] + 6, pts[-1][1] + 6], outline=(0, 128, 255), width=3)
    png = os.path.splitext(pgm)[0] + "_preview.png"
    im.save(png)
    print("preview", png, flush=True)

if __name__ == "__main__":
    main()
