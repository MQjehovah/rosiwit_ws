"""rebuild_grid.py - 从 PCD 找平后重建 2D 栅格地图。

步骤:
  1. 用最低一层点拟合地面平面 (z = a x + b y + c)
  2. 旋转点云使地面法向 = +Z, 并把地面平移到 z=0
  3. 保留 0.10..1.60 m 高度带的点 (墙/家具), 去掉地面
  4. 栅格化 (0.05) -> 去孤立点 -> 闭运算 -> 小连通域剔除
  5. 输出 PGM(0/254/205) + YAML + 找平后的 PCD
"""

import sys
import numpy as np
from PIL import Image


def read_pcd(path):
    with open(path, 'r') as f:
        header = {}
        while True:
            line = f.readline()
            if not line:
                break
            p = line.split()
            if not p:
                continue
            header[p[0]] = p[1:]
            if p[0] == 'DATA':
                break
        n = int(header['POINTS'][0])
        rows = []
        while len(rows) < n:
            line = f.readline()
            if not line:
                break
            parts = line.split()
            if len(parts) >= 3:
                rows.append([float(parts[0]), float(parts[1]), float(parts[2])])
    return np.array(rows, dtype=np.float64)


def write_pcd(path, pts):
    with open(path, 'w') as f:
        f.write("# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\n")
        f.write("FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n")
        f.write("WIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n" % len(pts))
        f.write("POINTS %d\nDATA ascii\n" % len(pts))
        for p in pts:
            f.write("%.4f %.4f %.4f\n" % (p[0], p[1], p[2]))


def fit_floor(pts):
    z = pts[:, 2]
    g0 = np.percentile(z, 2)
    floor = pts[z < g0 + 0.25]
    A = np.c_[floor[:, 0], floor[:, 1], np.ones(len(floor))]
    coef, *_ = np.linalg.lstsq(A, floor[:, 2], rcond=None)
    return coef  # a, b, c


def level(pts, coef):
    a, b, c = coef
    n = np.array([-a, -b, 1.0])
    n /= np.linalg.norm(n)
    axis = np.cross(n, [0.0, 0.0, 1.0])
    s = np.linalg.norm(axis)
    if s < 1e-9:
        R = np.eye(3)
    else:
        axis /= s
        angle = np.arccos(np.clip(n[2], -1, 1))
        K = np.array([[0, -axis[2], axis[1]],
                      [axis[2], 0, -axis[0]],
                      [-axis[1], axis[0], 0]])
        R = np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * (K @ K)
    out = pts @ R.T
    # 地面放到 z=0 (最低 2% 分位视为地面)
    out[:, 2] -= np.percentile(out[:, 2], 2)
    return out, R


def neighbors3(img):
    p = np.pad(img.astype(np.int32), 1, mode='constant')
    s = np.zeros_like(img, dtype=np.int32)
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            if dy == 0 and dx == 0:
                continue
            s += p[1 + dy:1 + dy + img.shape[0], 1 + dx:1 + dx + img.shape[1]]
    return s


def dilate(img):
    return neighbors3(img) > 0


def erode(img):
    p = np.pad(img, 1, mode='constant', constant_values=True)
    out = np.ones_like(img, dtype=bool)
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            out &= p[1 + dy:1 + dy + img.shape[0], 1 + dx:1 + dx + img.shape[1]]
    return out


def remove_small(occ, min_cells):
    h, w = occ.shape
    seen = np.zeros_like(occ, dtype=bool)
    out = occ.copy()
    for y in range(h):
        for x in range(w):
            if not occ[y, x] or seen[y, x]:
                continue
            stack = [(y, x)]
            seen[y, x] = True
            comp = []
            while stack:
                cy, cx = stack.pop()
                comp.append((cy, cx))
                for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    ny, nx = cy + dy, cx + dx
                    if 0 <= ny < h and 0 <= nx < w and occ[ny, nx] and not seen[ny, nx]:
                        seen[ny, nx] = True
                        stack.append((ny, nx))
            if len(comp) < min_cells:
                for cy, cx in comp:
                    out[cy, cx] = False
    return out


def main():
    pcd_in, out_dir = sys.argv[1], sys.argv[2]
    res = float(sys.argv[3]) if len(sys.argv) > 3 else 0.05
    min_h = float(sys.argv[4]) if len(sys.argv) > 4 else 0.10
    max_h = float(sys.argv[5]) if len(sys.argv) > 5 else 1.60

    pts = read_pcd(pcd_in)
    coef = fit_floor(pts)
    print("floor plane: z = %.4f x + %.4f y + %.3f" % tuple(coef))
    leveled, R = level(pts, coef)
    print("leveled: z range %.2f..%.2f" % (leveled[:, 2].min(), leveled[:, 2].max()))

    h = leveled[:, 2]
    keep = (h >= min_h) & (h <= max_h)
    p = leveled[keep]
    print("pcd=%d kept=%d" % (len(pts), len(p)))

    mn = p[:, :2].min(axis=0) - 0.30
    mx = p[:, :2].max(axis=0) + 0.30
    w = int(np.ceil((mx[0] - mn[0]) / res))
    hh = int(np.ceil((mx[1] - mn[1]) / res))
    occ = np.zeros((hh, w), dtype=bool)
    ix = ((p[:, 0] - mn[0]) / res).astype(int)
    iy = ((p[:, 1] - mn[1]) / res).astype(int)
    ok = (ix >= 0) & (ix < w) & (iy >= 0) & (iy < hh)
    occ[iy[ok], ix[ok]] = True

    nb = neighbors3(occ)
    occ = occ & (nb >= 1)
    occ = erode(dilate(occ))
    occ = remove_small(occ, 20)
    print("occupied cells:", int(occ.sum()))

    img = np.full((hh, w), 254, dtype=np.uint8)
    img[occ] = 0
    Image.fromarray(np.flipud(img)).save(out_dir + "/map.pgm")
    with open(out_dir + "/map.yaml", "w") as f:
        f.write("image: map.pgm\nresolution: %.3f\n" % res)
        f.write("origin: [%.5f, %.5f, 0.0]\n" % (mn[0], mn[1]))
        f.write("negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\nmode: trinary\n")
    write_pcd(out_dir + "/map.pcd", leveled)
    print("wrote %s/map.pgm (%dx%d), map.yaml, map.pcd" % (out_dir, w, hh))


if __name__ == '__main__':
    main()
