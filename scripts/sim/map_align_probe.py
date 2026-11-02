#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""判定: 静态地图与实时扫描是否对齐 (定位偏了 vs 地图里有伪障碍)。

做法:
  1. 打印车体周围 0.1m 的 ASCII 地图 (看是"墙"还是"一小块物体", 以及 footprint 位置)
  2. 把 /scan 用 TF(laser->map) 投到 map 系, 统计每个扫描点到最近"地图致命格"的距离
     - 中位数很小 (<=0.1m)  -> 地图与实景对齐 (那么 0.019m 那个就是地图里真有东西/伪障碍)
     - 中位数 0.3~1m        -> 定位/对齐偏了 (扫描的墙和地图的墙错位)
"""
from __future__ import print_function

import math
import sys
import time

import rospy
import tf2_ros
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

FP = [(-0.327, -0.359), (0.565, -0.359), (0.565, 0.359), (-0.327, 0.359)]


def load(msg):
    info = msg.info
    return (info.resolution, info.origin.position.x, info.origin.position.y,
            info.width, info.height, msg.data)


def cell_at(grid, wx, wy):
    res, ox, oy, w, h, data = grid
    gx = int((wx - ox) / res)
    gy = int((wy - oy) / res)
    if gx < 0 or gy < 0 or gx >= w or gy >= h:
        return None
    return data[gy * w + gx]


def main():
    rospy.init_node('map_align_probe', anonymous=True)
    st = {'map': None, 'diag': None, 'scan': None}
    rospy.Subscriber('/map', OccupancyGrid, lambda m: st.__setitem__('map', m), queue_size=1)
    rospy.Subscriber('/nav_diagnostics', String, lambda m: st.__setitem__('diag', m), queue_size=5)
    rospy.Subscriber('/scan', LaserScan, lambda m: st.__setitem__('scan', m), queue_size=3)
    t0 = time.time()
    while time.time() - t0 < 20 and (st['map'] is None or st['diag'] is None or st['scan'] is None):
        time.sleep(0.2)
    if st['map'] is None or st['diag'] is None:
        print('missing /map or /nav_diagnostics')
        return 1

    import json
    d = json.loads(st['diag'].data)
    p = d.get('odom_pose') or {}
    px, py = p.get('x', 0.0), p.get('y', 0.0)
    pyaw = math.radians(p.get('yaw', 0.0))
    print('pose(map) = (%.3f, %.3f, %.2f deg)' % (px, py, p.get('yaw', 0.0)))
    grid = load(st['map'])
    res, ox, oy, w, h, data = grid

    # 1) 车体周围 ASCII 地图 (0.1m 一格, +-1.5m)
    step = int(round(0.1 / res))
    r = 15
    print('--- 车体周围地图 (@%.1fm, #=lethal .=free ?=unknown, F=footprint) ---' % (r * 0.1))
    cx = int((px - ox) / res)
    cy = int((py - oy) / res)
    c, s = math.cos(pyaw), math.sin(pyaw)
    for dy in range(r, -r - 1, -step):
        row = ''
        for dx in range(-r, r + 1, step):
            gx, gy = cx + dx, cy + dy
            if gx < 0 or gy < 0 or gx >= w or gy >= h:
                row += ' '
                continue
            v = data[gy * w + gx]
            # 该格中心换算到 base 判断是否在 footprint 内
            wx = ox + (gx + 0.5) * res
            wy = oy + (gy + 0.5) * res
            bx = c * (wx - px) + s * (wy - py)
            by = -s * (wx - px) + c * (wy - py)
            in_fp = (-0.327 <= bx <= 0.565) and (-0.359 <= by <= 0.359)
            if in_fp:
                row += 'F'
            elif v < 0:
                row += '?'
            elif v >= 99:
                row += '#'
            else:
                row += '.'
        print('  ' + row)

    # 2) 扫描 vs 地图 距离统计
    try:
        tf = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf)
        sc = st['scan']
        t = tf.lookup_transform('map', sc.header.frame_id, rospy.Time(0), rospy.Duration(2.0))
    except Exception as e:  # noqa
        print('TF failed: %s' % e)
        return 1
    tx = t.transform.translation.x
    ty = t.transform.translation.y
    q = t.transform.rotation
    tyaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    ct, stt = math.cos(tyaw), math.sin(tyaw)
    dists = []
    ang = sc.angle_min
    for rng in sc.ranges:
        a = ang
        ang += sc.angle_increment
        if not (sc.range_min < rng < 3.5):
            continue
        lx, ly = rng * math.cos(a), rng * math.sin(a)
        wx = tx + ct * lx - stt * ly
        wy = ty + stt * lx + ct * ly
        # 找最近的地图致命格 (半径 1.2m)
        best = 1e9
        rr = int(1.2 / res)
        gx0 = int((wx - ox) / res)
        gy0 = int((wy - oy) / res)
        for gy in range(max(0, gy0 - rr), min(h, gy0 + rr + 1)):
            for gx in range(max(0, gx0 - rr), min(w, gx0 + rr + 1)):
                v = data[gy * w + gx]
                if v < 0 or v < 99:
                    continue
                d2 = ((ox + (gx + 0.5) * res) - wx) ** 2 + ((oy + (gy + 0.5) * res) - wy) ** 2
                if d2 < best:
                    best = d2
        if best < 1e9:
            dists.append(math.sqrt(best))
    if not dists:
        print('scan-vs-map: no matched points')
        return 0
    dists.sort()
    n = len(dists)
    print('scan-vs-map 点数=%d  中位数=%.3fm  p90=%.3fm  最大=%.3fm'
          % (n, dists[n // 2], dists[int(n * 0.9)], dists[-1]))
    print('判定: 中位数<=0.10 -> 地图与实景对齐(伪障碍/真障碍在图上); 中位数>=0.25 -> 定位/对齐偏了')
    return 0


if __name__ == '__main__':
    sys.exit(main())
