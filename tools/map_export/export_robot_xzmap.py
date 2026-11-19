#!/usr/bin/env python3
"""离线导出机器人 xzmap（不重跑建图）。

严格复刻 rosiwit_slam_ros2/src/io/robot_map_export.cpp 的约定：
  1) 关键帧: 与上一个关键帧的**净位移** >= kf_step (不是累计路径)
  2) 位姿扁平: z=0, roll=0, pitch=0, 只保留 yaw (rad)
  3) 角度毫弧度 (rad*1000)
  4) 关键帧点 = 关键帧局部坐标 (只按 yaw 旋转), z 减去地面高度
  5) 地面 z=0: 直方图最低强峰 (与 detectFloorZ 完全一致: -4..4m, 60 bins, 0.35*max)
  6) points_size = 所有关键帧点数之和; 空关键帧丢弃

用法:
  export_robot_xzmap.py <run_dir> [out.xzmap] [--kf-step 1.0]
run_dir 需要 map.pcd + odom_q.csv
"""
import json, math, os, sys
import numpy as np

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
    offs, o = [], 0
    for s in sizes: offs.append(o); o += s
    dt = np.dtype({"names": fields, "formats": ["<f4"]*len(fields),
                   "offsets": offs, "itemsize": o})
    a = np.frombuffer(raw, dtype=dt, count=n)
    return np.stack([a["x"], a["y"], a["z"]], 1).astype(np.float64)

def detect_floor_z(P):
    NB, LO, HI = 60, -4.0, 4.0
    idx = ((P[:, 2] - LO) / (HI - LO) * NB).astype(np.int64)
    ok = (idx >= 0) & (idx < NB)
    hist = np.bincount(idx[ok], minlength=NB)
    mx = hist.max() if len(hist) else 0
    if mx <= 0: return 0.0
    for i in range(NB):
        if hist[i] > 0.35 * mx:
            return LO + (i + 0.5) * (HI - LO) / NB
    return 0.0

def yaw_of(q):
    x, y, z, w = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

def main():
    run = sys.argv[1]
    out = sys.argv[2] if len(sys.argv) > 2 and not sys.argv[2].startswith("--") else os.path.join(run, "map.xzmap")
    kf_step = 1.0
    for i, a in enumerate(sys.argv):
        if a == "--kf-step" and i + 1 < len(sys.argv):
            kf_step = float(sys.argv[i + 1])

    P = read_pcd(os.path.join(run, "map.pcd"))
    print("points", len(P), flush=True)
    floor = detect_floor_z(P)
    print("floor_z %.3f" % floor, flush=True)

    d = np.genfromtxt(os.path.join(run, "odom_q.csv"), delimiter=",", names=True)
    order = np.argsort(d["t"])
    track = []
    for i in order:
        track.append((float(d["x"][i]), float(d["y"][i]),
                      yaw_of((d["qx"][i], d["qy"][i], d["qz"][i], d["qw"][i]))))
    print("track", len(track), "%.1fs" % (d["t"][order][-1] - d["t"][order][0]), flush=True)

    kfs = []
    for s in track:
        if not kfs or math.hypot(s[0] - kfs[-1][0], s[1] - kfs[-1][1]) >= kf_step:
            kfs.append(s)
    if len(kfs) < 2: kfs = track
    print("keyframes", len(kfs), "step %.2f m" % kf_step, flush=True)

    kp = np.array([[s[0], s[1]] for s in kfs], dtype=np.float64)
    nk = len(kp)
    idx = np.zeros(len(P), dtype=np.int64)
    for i in range(0, len(P), 4000):
        Q = P[i:i+4000, :2]
        d2 = ((Q[:, None, :] - kp[None, :, :]) ** 2).sum(2)
        idx[i:i+4000] = np.argmin(d2, axis=1)
    print("assigned", flush=True)

    order2 = np.argsort(idx, kind="stable")
    bounds = np.searchsorted(idx[order2], np.arange(nk + 1))
    counts = np.diff(bounds)
    kf_pts = []
    kf_poses = []
    for j in range(nk):
        if counts[j] == 0: continue
        sel = order2[bounds[j]:bounds[j+1]]
        x, y, yaw = kfs[j]
        cy, sy = math.cos(yaw), math.sin(yaw)
        dx = P[sel, 0] - x; dy = P[sel, 1] - y
        lx = cy * dx + sy * dy
        ly = -sy * dx + cy * dy
        lz = P[sel, 2] - floor
        pts = [{"x": int(round(a*1000)), "y": int(round(b*1000)), "z": int(round(c*1000))}
               for a, b, c in zip(lx, ly, lz)]
        kf_pts.append(pts)
        kf_poses.append({"x": int(round(x*1000)), "y": int(round(y*1000)),
                         "z": 0, "roll": 0, "pitch": 0,
                         "yaw": int(round(yaw*1000))})
    total = sum(len(p) for p in kf_pts)
    print("kept kf", len(kf_pts), "pts", total, flush=True)

    with open(out, "w", encoding="utf-8") as f:
        f.write("{\n")
        f.write('  "map_name": "%s",\n' % os.path.basename(out).replace(".xzmap", ""))
        f.write('  "points_size": %d,\n' % total)
        f.write('  "key_frame_size": %d,\n' % len(kf_poses))
        f.write('  "kf_frame_poses": [\n')
        for j, p in enumerate(kf_poses):
            f.write('    {"key_frame_id": %d, "x": %d, "y": %d, "z": 0, "roll": 0, "pitch": 0, "yaw": %d}%s\n'
                    % (j+1, p["x"], p["y"], p["yaw"], "," if j+1 < len(kf_poses) else ""))
        f.write('  ],\n')
        f.write('  "key_frame_points": [\n')
        for j, pts in enumerate(kf_pts):
            f.write('    {"key_frame_id": %d, "key_frame_point": [' % (j+1))
            f.write(",".join('{"x":%d,"y":%d,"z":%d}' % (p["x"], p["y"], p["z"]) for p in pts))
            f.write(']}%s\n' % ("," if j+1 < len(kf_pts) else ""))
        f.write('  ],\n')
        f.write('  "key_frame_points_2d": [\n')
        for j in range(len(kf_pts)):
            f.write('    {"key_frame_id": %d, "key_frame_point_2d": []}%s\n'
                    % (j+1, "," if j+1 < len(kf_pts) else ""))
        f.write('  ]\n}\n')
    print("wrote", out, "%.1f MB" % (os.path.getsize(out)/1e6))

if __name__ == "__main__":
    main()
