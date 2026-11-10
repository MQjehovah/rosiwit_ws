#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""导航噪点溯源: /scan 孤立点 + /local_map 孤立障碍格, 判断噪点来自雷达还是地图层."""
from __future__ import print_function
import math
import time
import collections

import rospy
import tf2_ros
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan

FP = (-0.262, 0.644, -0.338, 0.338)


def main():
    rospy.init_node('nav_noise_probe', anonymous=True)
    st = {'scan': None, 'lm': None, 'lc': None}
    rospy.Subscriber('/scan', LaserScan, lambda m: st.__setitem__('scan', m), queue_size=2)
    rospy.Subscriber('/local_map', OccupancyGrid, lambda m: st.__setitem__('lm', m), queue_size=1)
    rospy.Subscriber('/local_costmap', OccupancyGrid, lambda m: st.__setitem__('lc', m), queue_size=1)
    t0 = time.time()
    while time.time() - t0 < 15 and (st['scan'] is None or st['lm'] is None):
        time.sleep(0.05)
    sc = st['scan']
    if sc is None:
        print('no /scan')
        return 1

    tf = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf)
    tr = None
    for _ in range(60):
        try:
            tr = tf.lookup_transform('base_link', sc.header.frame_id, sc.header.stamp,
                                     rospy.Duration(0.3))
            break
        except Exception:
            time.sleep(0.05)
    if tr is None:
        tr = tf.lookup_transform('base_link', sc.header.frame_id, rospy.Time(0), rospy.Duration(2))
    tx, ty = tr.transform.translation.x, tr.transform.translation.y
    q = tr.transform.rotation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    c, s = math.cos(yaw), math.sin(yaw)

    pts = []
    ang = sc.angle_min
    hist = collections.Counter()
    inside = 0
    minr = 0
    for r in sc.ranges:
        a = ang
        ang += sc.angle_increment
        if not (sc.range_min < r < sc.range_max):
            continue
        minr += 1
        if r < 0.20:
            hist['0.0-0.2'] += 1
        elif r < 0.4:
            hist['0.2-0.4'] += 1
        elif r < 0.8:
            hist['0.4-0.8'] += 1
        elif r < 1.5:
            hist['0.8-1.5'] += 1
        elif r < 3.0:
            hist['1.5-3.0'] += 1
        else:
            hist['>3.0'] += 1
        lx, ly = r * math.cos(a), r * math.sin(a)
        bx = c * lx - s * ly + tx
        by = s * lx + c * ly + ty
        pts.append((bx, by))
        if FP[0] - 0.05 <= bx <= FP[1] + 0.05 and FP[2] - 0.05 <= by <= FP[3] + 0.05:
            inside += 1
    print('/scan beams=%d  inside_footprint(+5cm)=%d' % (minr, inside))
    print('range 分布: %s' % dict(hist))

    # 孤立点: 与最近邻 > 0.15m
    grid = {}
    for i, (x, y) in enumerate(pts):
        grid.setdefault((int(math.floor(x / 0.15)), int(math.floor(y / 0.15))), []).append(i)
    iso = []
    for i, (x, y) in enumerate(pts):
        gx, gy = int(math.floor(x / 0.15)), int(math.floor(y / 0.15))
        best = 1e9
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for j in grid.get((gx + dx, gy + dy), ()):
                    if j == i:
                        continue
                    d = math.hypot(pts[j][0] - x, pts[j][1] - y)
                    if d < best:
                        best = d
        if best > 0.15:
            iso.append((x, y, best))
    print('/scan 孤立点(最近邻>0.15m): %d / %d' % (len(iso), len(pts)))
    for x, y, d in sorted(iso, key=lambda v: -v[2])[:15]:
        print('   iso (%.2f, %.2f) nn=%.2f' % (x, y, d))

    # /local_map 障碍格 + 孤立性
    lm = st['lm']
    if lm is not None:
        info = lm.info
        res, ox, oy = info.resolution, info.origin.position.x, info.origin.position.y
        w, h, d = info.width, info.height, lm.data
        occ = [(ix, iy) for iy in range(h) for ix in range(w)
               if d[iy * w + ix] >= 65]
        occset = set(occ)
        iso_cells = []
        for (ix, iy) in occ:
            nb = sum(1 for dx in (-1, 0, 1) for dy in (-1, 0, 1)
                     if (dx or dy) and (ix + dx, iy + dy) in occset)
            if nb == 0:
                iso_cells.append((ix, iy))
        print('/local_map %dx%d res=%.3f occupied=%d 孤立障碍格=%d' % (
            w, h, res, len(occ), len(iso_cells)))
        # 孤立格是否有 scan 支撑
        supported = 0
        unsup = []
        for (ix, iy) in iso_cells:
            wx = ox + (ix + 0.5) * res
            wy = oy + (iy + 0.5) * res
            ok = False
            for (sx, sy) in pts:
                if abs(sx - wx) <= 0.2 and abs(sy - wy) <= 0.2:
                    ok = True
                    break
            if ok:
                supported += 1
            else:
                unsup.append((wx, wy))
        print('  孤立格有 /scan 支撑: %d, 无支撑: %d' % (supported, len(unsup)))
        for wx, wy in unsup[:10]:
            print('   ghost local_map cell (%.2f, %.2f)' % (wx, wy))

    lc = st['lc']
    if lc is not None:
        vals = collections.Counter(lc.data)
        print('/local_costmap 值分布(前6): %s' % vals.most_common(6))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
