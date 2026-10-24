#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""map_quality_report.py - 地图质量评估 (保存/使用前体检).

用法:
  rosrun rosiwit_slam_ros1 map_quality_report.py <map.yaml|目录|map.pgm> [选项]

评估项:
  1. 栅格: 尺寸/分辨率, free/occupied/unknown 占比
  2. 可通行性: 距离变换 + 连通性, 给定车体半宽时最大连通域占比,
     并给出仍能保持 >=95% 连通的最小半宽 (等价"最窄通道")
  3. 点云竖直覆盖 (地面以上障碍高度分层): 墙面 0.5~1.5m 层的格覆盖率,
     这正是"倾斜 3D 雷达旋转时对应点骤降"的根因指标
  4. 结论 PASS/WARN/FAIL (退出码 0/1), 便于流水线卡口

依赖: 仅标准库 (Python 2.7 / 3 均可).
"""
from __future__ import print_function

import math
import os
import sys
from collections import deque

BANDS = [(0.0, 0.2), (0.2, 0.5), (0.5, 1.0), (1.0, 1.5), (1.5, 99.0)]


def find_files(path):
    """把任意入口解析成 (pgm, yaml, pcd)."""
    if os.path.isdir(path):
        d = path
    else:
        d = os.path.dirname(os.path.abspath(path)) if not os.path.isdir(path) else path
    base = path
    if os.path.isdir(path):
        yaml_path = os.path.join(d, 'map.yaml')
    elif path.lower().endswith(('.yaml', '.yml')):
        yaml_path = path
    else:
        yaml_path = os.path.join(d, 'map.yaml')
    pgm_path = None
    pcd_path = None
    if os.path.isfile(yaml_path):
        for line in open(yaml_path):
            if line.strip().startswith('image:'):
                pgm_path = os.path.join(d, line.split(':', 1)[1].strip())
                break
    if not pgm_path or not os.path.isfile(pgm_path):
        cand = os.path.join(d, 'map.pgm')
        pgm_path = cand if os.path.isfile(cand) else None
    cand_pcd = os.path.join(d, 'map.pcd')
    if os.path.isfile(cand_pcd):
        pcd_path = cand_pcd
    return pgm_path, yaml_path if os.path.isfile(yaml_path) else None, pcd_path


def load_pgm(path):
    data = open(path, 'rb').read()
    parts, idx = [], 0
    while len(parts) < 4:
        while data[idx:idx + 1].isspace():
            idx += 1
        if data[idx:idx + 1] == b'#':
            while data[idx:idx + 1] not in (b'\n', b''):
                idx += 1
            continue
        s = idx
        while not data[idx:idx + 1].isspace():
            idx += 1
        parts.append(data[s:idx])
    idx += 1
    w, h = int(parts[1]), int(parts[2])
    img = data[idx:idx + w * h]
    occ = [[ord(img[y * w + x]) < 128 if not isinstance(img[y * w + x], int)
            else img[y * w + x] < 128 for x in range(w)] for y in range(h)]
    return w, h, occ


def load_yaml_meta(path):
    meta = {'resolution': 0.05, 'origin': [0.0, 0.0, 0.0]}
    if not path:
        return meta
    for line in open(path):
        s = line.strip()
        if s.startswith('resolution:'):
            meta['resolution'] = float(s.split(':', 1)[1])
        elif s.startswith('origin:'):
            try:
                meta['origin'] = [float(v) for v in
                                  s.split(':', 1)[1].strip().strip('[]').split(',')]
            except ValueError:
                pass
    return meta


def distance_transform(w, h, occ):
    INF = 10 ** 9
    dist = [[INF] * w for _ in range(h)]
    q = deque()
    for y in range(h):
        for x in range(w):
            if occ[y][x]:
                dist[y][x] = 0
                q.append((x, y))
    while q:
        x, y = q.popleft()
        d = dist[y][x] + 1
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1),
                       (1, 1), (1, -1), (-1, 1), (-1, -1)):
            nx, ny = x + dx, y + dy
            if 0 <= nx < w and 0 <= ny < h and dist[ny][nx] > d:
                dist[ny][nx] = d
                q.append((nx, ny))
    return dist


def largest_component(w, h, dist, thresh_cells):
    seen = [[False] * w for _ in range(h)]
    best = total = 0
    for y0 in range(h):
        for x0 in range(w):
            if seen[y0][x0] or dist[y0][x0] < thresh_cells:
                continue
            q = deque([(x0, y0)])
            seen[y0][x0] = True
            cnt = 0
            while q:
                x, y = q.popleft()
                cnt += 1
                for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    nx, ny = x + dx, y + dy
                    if 0 <= nx < w and 0 <= ny < h and not seen[ny][nx] \
                            and dist[ny][nx] >= thresh_cells:
                        seen[ny][nx] = True
                        q.append((nx, ny))
            best = max(best, cnt)
            total += cnt
    return best, total


def load_pcd_xyz(path, max_points=4000000):
    pts = []
    with open(path) as f:
        fmt = None
        for line in f:
            if line.startswith('DATA'):
                fmt = line.split()[1].strip()
                break
        if fmt != 'ascii':
            print('  [pcd] binary format: z 分层统计跳过 (用 pcl_convert_pcd_ascii_binary 转 ascii)')
            return None
        for line in f:
            v = line.split()
            if len(v) >= 3:
                try:
                    pts.append((float(v[0]), float(v[1]), float(v[2])))
                except ValueError:
                    continue
                if len(pts) >= max_points:
                    print('  [pcd] 点数超过上限 %d, 截断统计' % max_points)
                    break
    return pts


def main():
    argv = sys.argv[1:]
    if not argv:
        print(__doc__)
        return 2
    path = argv[0]
    half_width = 0.39
    min_connected = 0.95
    for i, a in enumerate(argv):
        if a == '--robot-half-width' and i + 1 < len(argv):
            half_width = float(argv[i + 1])

    pgm, yaml_path, pcd = find_files(path)
    print('== 地图质量报告 ==')
    print('地图: %s' % (pgm or path))
    if not pgm:
        print('FAIL: 找不到 map.pgm')
        return 1
    meta = load_yaml_meta(yaml_path)
    res = float(meta['resolution'])
    w, h, occ = load_pgm(pgm)
    occ_cells = sum(1 for y in range(h) for x in range(w) if occ[y][x])
    free_cells = w * h - occ_cells
    print('栅格: %dx%d  res=%.2f m  尺寸 %.1fx%.1f m  占用 %d (%.1f%%)'
          % (w, h, res, w * res, h * res, occ_cells, 100.0 * occ_cells / (w * h)))

    dist = distance_transform(w, h, occ)
    need = int(math.ceil(half_width / res))
    best, total = largest_component(w, h, dist, need)
    conn = float(best) / max(1, total)
    print('可通行性: 车体半宽 %.2f m -> 最大连通域 %.1f%%%s'
          % (half_width, 100.0 * conn, '' if conn >= min_connected else '  <-- 有区域不连通'))
    # 保持 95% 连通的最小半宽 (二分)
    lo, hi = 0.0, 1.0
    for _ in range(8):
        mid = (lo + hi) * 0.5
        b, t = largest_component(w, h, dist, max(1, int(math.ceil(mid / res))))
        if float(b) / max(1, t) >= min_connected:
            lo = mid
        else:
            hi = mid
    print('最窄通道: 半宽 ~%.2f m (宽 ~%.2f m)' % (lo, lo * 2.0))

    wall_cov = None
    if pcd:
        pts = load_pcd_xyz(pcd)
        if pts:
            bands = [0] * len(BANDS)
            wall_cells = set()
            for (x, y, z) in pts:
                for bi, (z0, z1) in enumerate(BANDS):
                    if z0 <= z < z1:
                        bands[bi] += 1
                        break
                if 0.3 <= z < 1.5:
                    wall_cells.add((int(math.floor(x)), int(math.floor(y))))
            print('点云: %d pts  高度分层:' % len(pts))
            for (z0, z1), c in zip(BANDS, bands):
                print('   z %4.1f~%-4.1f : %7d (%.1f%%)'
                      % (z0, min(z1, 9.9), c, 100.0 * c / len(pts)))
            # 分母用 2D 栅格的真实障碍格(1m 聚合): 用点云投影会把地面算进去, 覆盖率被稀释
            occ_1m = set()
            for yy in range(h):
                for xx in range(w):
                    if not occ[yy][xx]:
                        continue
                    wx = meta['origin'][0] + (xx + 0.5) * res
                    wy = meta['origin'][1] + (h - 1 - yy + 0.5) * res
                    occ_1m.add((int(math.floor(wx)), int(math.floor(wy))))
            hit = len([c for c in occ_1m if c in wall_cells])
            wall_cov = float(hit) / max(1, len(occ_1m))
            print('墙面覆盖: 0.3~1.5m 有点的 1m 格 / 栅格障碍 1m 格 = %d/%d = %.1f%%'
                  % (hit, len(occ_1m), 100.0 * wall_cov))
            print('  (低于 40% 说明竖直结构缺失, 倾斜 3D 雷达旋转时容易掉匹配)')

    verdict = 'PASS'
    if conn < min_connected:
        verdict = 'FAIL'
    elif wall_cov is not None and wall_cov < 0.40:
        verdict = 'WARN'
    print('结论: %s' % verdict)
    return 0 if verdict != 'FAIL' else 1


if __name__ == '__main__':
    sys.exit(main())
