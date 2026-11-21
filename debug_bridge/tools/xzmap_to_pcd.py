#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
xkmap -> 标准 PCD（binary，FIELDS x y z，float32）。

xzmap 是机器人实际载入的地图（关键帧位姿 + 机体系点云，mm/mrad）。
这里按位姿把点变换到世界系，体素降采样，再写标准 PCD：

  python3 xzmap_to_pcd.py map.xzmap -o 1楼.pcd --voxel 0.3 --max 800000

可选：
  --ascii      写 ASCII PCD（更大，但纯文本）
  --stream     用 ijson 低内存解析（需 pip install ijson）
  --raw        不做体素/上限处理

PCD 是标准 binary（DATA binary），RViz / PCL / debug_hmi 都能直接读。
"""

import argparse
import math
import os
import sys
import time

try:
    import numpy as np
except ImportError:
    print("需要 numpy: pip install numpy", file=sys.stderr)
    raise SystemExit(2)


def log(msg):
    print(msg, flush=True)


def rot_zyx(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    # R = Rz(yaw) * Ry(pitch) * Rx(roll)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=np.float64,
    )


def load_json(path):
    import json

    log("reading %s (%.1f MB) ..." % (path, os.path.getsize(path) / 1e6))
    t0 = time.time()
    with open(path, "r") as f:
        data = json.load(f)
    log("parsed in %.1fs" % (time.time() - t0))
    return data


def load_streaming(path):
    try:
        import ijson
    except ImportError:
        return None
    log("streaming parse with ijson ...")
    t0 = time.time()
    poses = []
    with open(path, "rb") as f:
        for item in ijson.items(f, "kf_frame_poses.item"):
            poses.append(item)
    points = []
    with open(path, "rb") as f:
        for item in ijson.items(f, "key_frame_points.item"):
            points.append(item)
    log("parsed in %.1fs" % (time.time() - t0))
    return {"kf_frame_poses": poses, "key_frame_points": points}


def collect_world_points(data):
    poses = data.get("kf_frame_poses") or []
    kfs = data.get("key_frame_points") or []
    if not poses or not kfs:
        raise SystemExit("xzmap 缺少 kf_frame_poses / key_frame_points")

    pose_by_id = {}
    for p in poses:
        pose_by_id[int(p.get("key_frame_id", 0))] = p

    chunks = []
    total = 0
    for kf in kfs:
        kid = int(kf.get("key_frame_id", 0))
        pose = pose_by_id.get(kid)
        if pose is None:
            continue
        t = np.array(
            [pose.get("x", 0.0), pose.get("y", 0.0), pose.get("z", 0.0)], dtype=np.float64
        ) / 1000.0
        rpy = np.array(
            [pose.get("roll", 0.0), pose.get("pitch", 0.0), pose.get("yaw", 0.0)], dtype=np.float64
        ) / 1000.0
        rot = rot_zyx(rpy[0], rpy[1], rpy[2])
        pts = kf.get("key_frame_point") or []
        if not pts:
            continue
        arr = np.fromiter(
            (
                v
                for pt in pts
                for v in (
                    pt.get("x", 0.0) / 1000.0,
                    pt.get("y", 0.0) / 1000.0,
                    pt.get("z", 0.0) / 1000.0,
                )
            ),
            dtype=np.float64,
            count=len(pts) * 3,
        ).reshape(-1, 3)
        chunks.append(arr.dot(rot.T) + t)
        total += len(arr)
        if total % 500000 < len(arr):
            log("  %d keyframes, %d points ..." % (len(chunks), total))
    if not chunks:
        raise SystemExit("没有可用点")
    out = np.concatenate(chunks, axis=0)
    log("world points: %d" % len(out))
    return out


def voxel_downsample(points, voxel, max_points):
    if voxel <= 0:
        sampled = points
    else:
        keys = np.floor(points / voxel).astype(np.int64)
        mn = keys.min(axis=0)
        mx = keys.max(axis=0)
        dims = (mx - mn + 1).astype(np.int64)
        flat = (
            (keys[:, 0] - mn[0])
            + (keys[:, 1] - mn[1]) * dims[0]
            + (keys[:, 2] - mn[2]) * dims[0] * dims[1]
        )
        _, idx = np.unique(flat, return_index=True)
        sampled = points[idx]
        log("voxel %.2fm: %d -> %d points" % (voxel, len(points), len(sampled)))
    if max_points and len(sampled) > max_points:
        stride = int(math.ceil(float(len(sampled)) / max_points))
        sampled = sampled[::stride]
        log("max %d: stride %d -> %d points" % (max_points, stride, len(sampled)))
    return sampled


def write_pcd(path, points, ascii_mode=False):
    n = len(points)
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z\n"
        "SIZE 4 4 4\n"
        "TYPE F F F\n"
        "COUNT 1 1 1\n"
        "WIDTH %d\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        "POINTS %d\n"
        "DATA %s\n" % (n, n, "ascii" if ascii_mode else "binary")
    )
    with open(path, "wb") as f:
        f.write(header.encode("ascii"))
        if ascii_mode:
            for p in points:
                f.write(("%.4f %.4f %.4f\n" % (p[0], p[1], p[2])).encode("ascii"))
        else:
            f.write(np.ascontiguousarray(points, dtype="<f4").tobytes())
    log("wrote %s: %d points, %.1f MB" % (path, n, os.path.getsize(path) / 1e6))


def main():
    ap = argparse.ArgumentParser(description="xzmap -> standard PCD (x y z float32)")
    ap.add_argument("input")
    ap.add_argument("-o", "--output", default=None, help="输出 .pcd")
    ap.add_argument("--voxel", type=float, default=0.3, help="体素边长（m），0=不降采样")
    ap.add_argument("--max", type=int, default=800000, help="最多点数")
    ap.add_argument("--ascii", action="store_true", help="写 ASCII PCD")
    ap.add_argument("--stream", action="store_true", help="用 ijson 低内存解析（需安装 ijson）")
    ap.add_argument("--raw", action="store_true", help="不降采样/不限点数")
    args = ap.parse_args()

    out = args.output or os.path.splitext(args.input)[0] + ".pcd"
    data = load_streaming(args.input) if args.stream else None
    if data is None:
        data = load_json(args.input)

    points = collect_world_points(data)
    if not args.raw:
        points = voxel_downsample(points, args.voxel, args.max)
    write_pcd(out, points, args.ascii)


if __name__ == "__main__":
    main()
