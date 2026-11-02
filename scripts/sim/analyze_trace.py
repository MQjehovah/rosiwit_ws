#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""分析 trace CSV: 安全状态何时变、门前速度、路径与车体相对门中心的横向偏移.

用法: python2 analyze_trace.py trace.csv [door_y=2.9]
输出全 ASCII, 方便在 WSL 里看。
"""
from __future__ import print_function

import math
import sys


def load(csv_path):
    rows = []
    with open(csv_path) as f:
        head = f.readline().strip().split(',')
        for line in f:
            p = line.strip().split(',')
            if len(p) != len(head):
                continue
            d = dict(zip(head, p))
            try:
                rows.append({
                    't': float(d['t']), 'x': float(d['x']), 'y': float(d['y']),
                    'yaw': float(d['yaw']), 'state': d['state'],
                    'gs': d['goal_status'], 'safety': d['safety'],
                    'scale': float(d['scale']), 'v': float(d['v']),
                    'w': float(d['w']),
                })
            except (KeyError, ValueError):
                continue
    return rows


def at_y(rows, y_target, tol=0.06):
    """返回 y 最接近 y_target 的那一拍 (只在移动过之后找)."""
    best = None
    for r in rows:
        if abs(r['y'] - y_target) <= tol:
            if best is None or abs(r['y'] - y_target) < abs(best['y'] - y_target):
                best = r
    return best


def main():
    csv_path = sys.argv[1]
    door_y = float(sys.argv[2]) if len(sys.argv) > 2 else 2.9
    door_half = float(sys.argv[3]) if len(sys.argv) > 3 else 0.39
    rows = load(csv_path)
    if not rows:
        print('no rows')
        return 1

    hist = {}
    for r in rows:
        hist[r['safety']] = hist.get(r['safety'], 0) + 1
    print('safety_hist:', ' '.join('%s=%d' % (k, v) for k, v in sorted(hist.items())))
    gs_hist = {}
    for r in rows:
        gs_hist[r['gs']] = gs_hist.get(r['gs'], 0) + 1
    print('goal_status_hist:', ' '.join('%s=%d' % (k, v) for k, v in sorted(gs_hist.items())))
    print('duration_s=%.1f y_range=%.2f..%.2f' % (rows[-1]['t'], rows[0]['y'], rows[-1]['y']))

    # 第一次进入非 PASS
    first = None
    for r in rows:
        if r['safety'] != 'PASS':
            first = r
            break
    if first is None:
        print('never_left_PASS=True')
    else:
        print('first_nonpass: t=%.2f y=%.3f x=%.3f dist_to_door=%.3f state=%s '
              'scale=%.3f v=%.3f w=%.3f' % (
                  first['t'], first['y'], first['x'], door_y - first['y'],
                  first['safety'], first['scale'], first['v'], first['w']))

    # 速度 vs 门前距离 (每 0.5m 一格)
    print('--- approach profile (y bin: mean |v|, min scale, states) ---')
    bins = {}
    for r in rows:
        if r['y'] < 0.2:
            continue
        k = int(r['y'] / 0.5)
        b = bins.setdefault(k, {'n': 0, 'v': 0.0, 'scale': 1.0, 'st': set()})
        b['n'] += 1
        b['v'] += abs(r['v'])
        b['scale'] = min(b['scale'], r['scale'])
        b['st'].add(r['safety'])
    for k in sorted(bins):
        b = bins[k]
        print('  y %.1f-%.1f  mean_v=%.3f min_scale=%.2f  %s' % (
            k * 0.5, k * 0.5 + 0.5, b['v'] / b['n'], b['scale'],
            ','.join(sorted(b['st']))))

    # 门平面附近的车体横向位置
    for target in (door_y - 0.5, door_y - 0.2, door_y, door_y + 0.2):
        r = at_y(rows, target)
        if r is None:
            print('  near y=%.2f: no sample' % target)
        else:
            print('  near y=%.2f: x=%.3f yaw_deg=%.1f v=%.3f safety=%s scale=%.2f' % (
                target, r['x'], r['yaw'], r['v'], r['safety'], r['scale']))

    # 路径在门平面处的横向位置 (居中是否真的把路径放到 0)
    path_file = csv_path + '.path'
    try:
        pts = []
        with open(path_file) as pf:
            pf.readline()
            for line in pf:
                a = line.split()
                if len(a) == 2:
                    pts.append((float(a[0]), float(a[1])))
    except IOError:
        pts = []
    if len(pts) < 2:
        print('path: not captured')
        pts = []
    if pts:
        xs = [x for x, y in pts if abs(y - door_y) < 0.15]
        print('path_pts=%d  path_x_at_door=%s' % (
            len(pts), ('%.3f' % (sum(xs) / len(xs))) if xs else 'none'))
        near = [x for x, y in pts if abs(y - door_y) < 1.0]
        if near:
            print('path_x_mean_near_door=%.3f max_abs=%.3f' % (
                sum(near) / len(near), max(abs(v) for v in near)))

    # ---- 物理判定: 门带内车体角点是否越过门框 ----
    # Gazebo 碰撞体 = 0.87 x 0.70 (x 偏移 0.119) -> x -0.316..0.554, y +-0.35
    # (导航声明 footprint 是 0.66 宽, 比碰撞体窄 4cm; 两个口径都算)
    # 门洞 0.80m (碰撞体 0.70 + 10cm) -> 半宽 0.40
    bodies = [
        ('collision_0.87x0.70', [(-0.316, -0.35), (0.554, -0.35), (0.554, 0.35), (-0.316, 0.35)]),
        ('footprint_906x676', [(-0.262, -0.338), (0.644, -0.338), (0.644, 0.338), (-0.262, 0.338)]),
    ]
    def judge(use_gt, tag):
        for name, fp in bodies:
            worst = 0.0
            samples = 0
            for r in rows:
                try:
                    if use_gt:
                        px, py, pyaw = float(r['gt_x']), float(r['gt_y']), float(r['gt_yaw'])
                        if math.isnan(px):
                            continue
                    else:
                        px, py, pyaw = r['x'], r['y'], r['yaw']
                except (KeyError, ValueError):
                    continue
                if py < 2.80 or py > 3.30:
                    continue
                c = math.cos(math.radians(pyaw))
                s = math.sin(math.radians(pyaw))
                for lx, ly in fp:
                    wx = px + c * lx - s * ly
                    wy = py + s * lx + c * ly
                    if 2.90 <= wy <= 3.10:
                        worst = max(worst, abs(wx))
                        samples += 1
            if samples == 0:
                verdict = 'NOT_REACHED'
            elif worst <= door_half:
                verdict = 'PASS'
            else:
                verdict = 'CLIP(+%.0fmm)' % ((worst - door_half) * 1000)
            print('gate_verdict%s[%s]: worst_corner_x=%.4f (half_gap %.2f, n=%d) -> %s' % (
                tag, name, worst, door_half, samples, verdict))

    # 判定两个口径: odom (导航自己以为的位姿) 与 Gazebo 真值 (物理真相)。
    # 两者差距 = 定位/里程计误差, 在 5cm 余量的窄门里是决定性的。
    judge(False, '')
    judge(True, '_gt')
    return 0


if __name__ == '__main__':
    sys.exit(main())
