#!/usr/bin/env python3
"""把一次建图结果导出成机器人（xz_3d_slam）可直接使用的地图包。

背景：机器人内部 SLAM 的 xzmap 有几条硬约定，任一条不对，定位就会"看起来贴墙、
实际 inlier 上不去"。这些约定是在真机上逐项验证出来的（见 docs/robot_map_export.md）：

1. 关键帧点必须是**关键帧局部坐标**
     map_updater.cpp:581     保存: p_local = T⁻¹ · p_world
     nano_gicp_localization.cpp:1646  载入: p_world = local2global(p_local, pose)
2. 角度是**毫弧度**（不是毫度）
     map_updater.cpp:603-605  pose_6d.yaw(rad) * 1000
     nano_gicp_localization.cpp:9336     tf.yaw * 0.001  （pcl::getTransformation 要弧度）
3. 关键帧位姿是扁平的：z=0、roll=0、pitch=0，只保留 yaw
     mapping_conf.json: "fix_z": true, "fix_rotation": true
4. 地图坐标以**地面为 z=0**
5. 关键帧间距参考 keyframeMeterGap = 1.0 m

用法:
    export_robot_map.py <run_dir> [--kf-step 1.0] [--out-prefix map_robot] [--zip out.zip]

run_dir 需要包含:
    map.pcd      建图输出的三维点云（世界坐标）
    odom_bag     rosbag2，含 /lio_odom（位姿 + 四元数）
    map.pgm      （可选）2D 栅格
    map.yaml     （可选）2D 栅格参数
"""
import argparse
import json
import math
import os
import zipfile

import numpy as np

try:
    from rosbags.rosbag2 import Reader
    from rosbags.typesys import Stores, get_typestore
except ImportError as e:  # pragma: no cover
    raise SystemExit("需要 rosbags: pip install rosbags") from e


def read_pcd(path):
    with open(path, "rb") as f:
        fields = sizes = n = None
        while True:
            line = f.readline().decode("ascii", "ignore").strip()
            if not line:
                continue
            k, _, v = line.partition(" ")
            if k == "FIELDS":
                fields = v.split()
            elif k == "SIZE":
                sizes = [int(x) for x in v.split()]
            elif k == "POINTS":
                n = int(v)
            elif k == "DATA":
                break
        raw = f.read()
    offs, o = [], 0
    for s in sizes:
        offs.append(o)
        o += s
    dt = np.dtype({"names": fields, "formats": ["<f4"] * len(fields),
                   "offsets": offs, "itemsize": o})
    a = np.frombuffer(raw, dtype=dt, count=n)
    return np.stack([a["x"], a["y"], a["z"]], 1).astype(np.float64)


def write_pcd(path, P):
    pts = np.ascontiguousarray(P, dtype=np.float32)
    rec = np.zeros(len(pts), dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4")])
    rec["x"], rec["y"], rec["z"] = pts[:, 0], pts[:, 1], pts[:, 2]
    with open(path, "wb") as f:
        f.write(b"VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n")
        f.write(b"WIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA binary\n"
                % (len(pts), len(pts)))
        f.write(rec.tobytes())


def load_odom(bag_dir):
    ts = get_typestore(Stores.ROS2_HUMBLE)
    rows = []
    with Reader(bag_dir) as r:
        conns = [c for c in r.connections if c.topic in ("/lio_odom", "/odom")]
        for c, _, raw in r.messages(connections=conns):
            m = ts.deserialize_cdr(raw, c.msgtype)
            p = m.pose.pose.position
            q = m.pose.pose.orientation
            t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            rows.append((t, p.x, p.y, p.z, q.x, q.y, q.z, q.w))
    A = np.array(rows)
    return A[np.argsort(A[:, 0])]


def yaw_of(q):
    x, y, z, w = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def detect_floor_z(P, origin_xy, radius=12.0, min_pts=200):
    """地面高度 = 地图原点(第一个关键帧)附近 radius 内点云 z 的 2% 分位。

    不要用全局直方图取"最低强峰": 图一旦有 z 漂移(长坡道/回环), 峰值会被漂移区
    拉走。2026-11-19 genz 图实测取到 -3.93, 整图被抬高 3.93 m, 原点地面变 +2.36,
    重定位 fitness 只有 0.68, 永远达不到 th_fitness_relocalization=0.89,
    表现为一直"定位丢失"。
    """
    if origin_xy is None:
        return float(np.percentile(P[:, 2], 2))
    dx = P[:, 0] - origin_xy[0]
    dy = P[:, 1] - origin_xy[1]
    m = (dx * dx + dy * dy) <= radius * radius
    if m.sum() < min_pts:
        return float(np.percentile(P[:, 2], 2))
    return float(np.percentile(P[m, 2], 2))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run_dir")
    ap.add_argument("--kf-step", type=float, default=1.0)
    ap.add_argument("--out-prefix", default="map_robot")
    ap.add_argument("--zip", default="")
    ap.add_argument("--no-z-align", action="store_true",
                    help="不做地面 z=0 平移（默认会做）")
    args = ap.parse_args()

    run = args.run_dir
    P = read_pcd(os.path.join(run, "map.pcd"))
    print("points", len(P))

    A = load_odom(os.path.join(run, "odom_bag"))
    A[:, 0] -= A[0, 0]
    print("odom %d frames, %.1f s" % (len(A), A[-1, 0]))

    kf = [A[0]]
    last, acc = A[0], 0.0
    for r in A[1:]:
        acc += math.hypot(r[1] - last[1], r[2] - last[2])
        if acc >= args.kf_step:
            kf.append(r)
            last, acc = r, 0.0
    if kf[-1] is not A[-1]:
        kf.append(A[-1])
    print("keyframes", len(kf), "step %.2f m" % args.kf_step)

    # 地面 z 锚点 = 地图原点(第一个关键帧)附近 12 m 内点云的 2% 分位
    if not args.no_z_align:
        floor = detect_floor_z(P, (kf[0][1], kf[0][2]))
        P[:, 2] -= floor
        print("ground z = %.3f -> 原点地面平移到 0" % floor)

    px = np.array([k[1] for k in kf])
    py = np.array([k[2] for k in kf])
    yaws = np.array([yaw_of(k[4:8]) for k in kf])

    # 关键帧 = 半径内整片点云(自适应体素压到 KF_MAX_PTS), 不是"最近邻格子".
    # 机器人侧 kf_map_filter_num=2 会隔一个丢一个关键帧: 每帧只带 1m 格子时,
    # 丢一半等于在地图上开 1m 宽的沟(覆盖-49%, fitness 卡 0.78); 整片点云后 0.96.
    KF_RADIUS = 8.0
    KF_MAX_PTS = 2400

    def pick_points(j):
        dx = P[:, 0] - px[j]
        dy = P[:, 1] - py[j]
        Q = P[(dx * dx + dy * dy) <= KF_RADIUS * KF_RADIUS]
        if len(Q) == 0:
            return Q
        leaf = 0.30
        S = Q
        for _ in range(6):
            key = np.floor(Q / leaf).astype(np.int64)
            _, first = np.unique(key, axis=0, return_index=True)
            S = Q[np.sort(first)]
            if len(S) <= KF_MAX_PTS:
                break
            leaf *= 1.35
        return S[:KF_MAX_PTS]

    out_pcd = os.path.join(run, args.out_prefix + ".pcd")
    write_pcd(out_pcd, P)

    out_xz = os.path.join(run, args.out_prefix + ".xzmap")
    with open(out_xz, "w", encoding="utf-8") as f:
        f.write("{\n")
        f.write('  "map_name": "%s",\n' % os.path.basename(run.rstrip("/")))
        f.write('  "points_size": %d,\n' % len(P))
        f.write('  "key_frame_size": %d,\n' % len(kf))
        f.write('  "kf_frame_poses": [\n')
        for j in range(len(kf)):
            # 位姿：z=0 / roll=pitch=0（fix_z、fix_rotation）；角度单位 = 毫弧度
            f.write('    {"key_frame_id": %d, "x": %d, "y": %d, "z": 0, '
                    '"roll": 0, "pitch": 0, "yaw": %d}%s\n'
                    % (j + 1, round(px[j] * 1000), round(py[j] * 1000),
                       round(yaws[j] * 1000),
                       "," if j + 1 < len(kf) else ""))
        f.write("  ],\n")
        f.write('  "key_frame_points": [\n')
        for j in range(len(kf)):
            sel = pick_points(j)
            cy, sy = math.cos(yaws[j]), math.sin(yaws[j])
            d = sel - np.array([px[j], py[j], 0.0]) if len(sel) else sel.reshape(0, 3)
            # 关键帧局部坐标（只按 yaw 旋转；由 local2global 还原）
            lx = cy * d[:, 0] + sy * d[:, 1]
            ly = -sy * d[:, 0] + cy * d[:, 1]
            parts = ['{"x":%d,"y":%d,"z":%d}' % (round(a * 1000), round(b * 1000), round(c * 1000))
                     for a, b, c in zip(lx, ly, d[:, 2])]
            f.write('    {"key_frame_id": %d, "key_frame_point": [' % (j + 1))
            f.write(",".join(parts))
            f.write("]}%s\n" % ("," if j + 1 < len(kf) else ""))
        f.write("  ],\n")
        f.write('  "key_frame_points_2d": [\n')
        for j in range(len(kf)):
            f.write('    {"key_frame_id": %d, "key_frame_point_2d": []}%s\n'
                    % (j + 1, "," if j + 1 < len(kf) else ""))
        f.write("  ]\n}\n")
    print("wrote", out_xz, "%.1f MB" % (os.path.getsize(out_xz) / 1e6))

    if args.zip:
        with zipfile.ZipFile(args.zip, "w", zipfile.ZIP_DEFLATED, compresslevel=6) as z:
            z.write(out_pcd, "map.pcd")
            z.write(out_xz, "map.xzmap")
            for name in ("map.pgm", "map.yaml"):
                p = os.path.join(run, name)
                if os.path.exists(p):
                    z.write(p, name)
            z.writestr("meta.txt",
                       "points=%d kf=%d kf_step=%.2f yaw_unit=mrad poses=flat_z0\n"
                       % (len(P), len(kf), args.kf_step))
        print("wrote", args.zip, "%.1f MB" % (os.path.getsize(args.zip) / 1e6))


if __name__ == "__main__":
    main()
