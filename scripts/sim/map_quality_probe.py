#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""静态栅格地图质量探针 (判定"地图是否胀了/生成有问题")。

三个判据:
 1) 值分布: free(0) / occupied(>=65) / unknown(-1) 的比例
 2) 车体周围全分辨率 ASCII (1 格 1 字符), 叠加 footprint 轮廓
 3) **射线自由性**: 激光从车体打到障碍, 沿途应为 free。
    若沿途大量 occupied/unknown -> 地图胀了 (unknown 当占用 / 障碍膨胀 / 高度过滤错)
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

FP = [(-0.262, -0.338), (0.644, -0.338), (0.644, 0.338), (-0.262, 0.338)]


def main():
    rospy.init_node('map_quality_probe', anonymous=True)
    st = {'map': None, 'diag': None, 'scan': None}
    rospy.Subscriber('/map', OccupancyGrid, lambda m: st.__setitem__('map', m), queue_size=1)
    rospy.Subscriber('/nav_diagnostics', String, lambda m: st.__setitem__('diag', m), queue_size=5)
    rospy.Subscriber('/scan', LaserScan, lambda m: st.__setitem__('scan', m), queue_size=3)
    t0 = time.time()
    while time.time() - t0 < 25 and (st['map'] is None or st['diag'] is None or st['scan'] is None):
        time.sleep(0.2)
    if st['map'] is None or st['diag'] is None:
        print('missing /map or /nav_diagnostics')
        return 1

    import json
    d = json.loads(st['diag'].data)
    p = d.get('odom_pose') or {}
    px, py = p.get('x', 0.0), p.get('y', 0.0)
    pyaw = math.radians(p.get('yaw', 0.0))

    m = st['map']
    info = m.info
    res, ox, oy = info.resolution, info.origin.position.x, info.origin.position.y
    w, h, data = info.width, info.height, m.data
    print('map: %dx%d res=%.3f origin=(%.2f,%.2f) frame=%s' % (w, h, res, ox, oy, m.header.frame_id))
    print('pose: (%.3f, %.3f, %.2f deg)' % (px, py, p.get('yaw', 0.0)))

    # 1) 值分布
    free = occ = unk = other = 0
    for v in data:
        if v < 0:
            unk += 1
        elif v >= 65:
            occ += 1
        elif v == 0:
            free += 1
        else:
            other += 1
    n = float(len(data))
    print('值分布: free=%.1f%%  occupied(>=65)=%.1f%%  unknown=%.1f%%  其它(1..64)=%.1f%%'
          % (100 * free / n, 100 * occ / n, 100 * unk / n, 100 * other / n))

    def cell(wx, wy):
        gx = int((wx - ox) / res)
        gy = int((wy - oy) / res)
        if gx < 0 or gy < 0 or gx >= w or gy >= h:
            return None
        return data[gy * w + gx]

    print('车体位姿格的值: %s  (0=free, >=65=occupied, -1=unknown)' % cell(px, py))

    # 2) 车体周围全分辨率 ASCII (0.05m/格 -> 半径 ~0.8m)
    rad_cells = int(0.8 / res)
    c, s = math.cos(pyaw), math.sin(pyaw)
    print('--- 车体周围 (1 格=%.2fm, 上=+y) F=footprint #=occ ?=unknown .=free ---' % res)
    for dy in range(rad_cells, -rad_cells - 1, -1):
        row = ''
        for dx in range(-rad_cells, rad_cells + 1):
            gx = int((px - ox) / res) + dx
            gy = int((py - oy) / res) + dy
            if gx < 0 or gy < 0 or gx >= w or gy >= h:
                row += ' '
                continue
            wx = ox + (gx + 0.5) * res
            wy = oy + (gy + 0.5) * res
            bx = c * (wx - px) + s * (wy - py)
            by = -s * (wx - px) + c * (wy - py)
            if (-0.262 <= bx <= 0.644) and (-0.338 <= by <= 0.338):
                row += 'F'
                continue
            v = data[gy * w + gx]
            row += '?' if v < 0 else ('#' if v >= 65 else ('.' if v == 0 else '+'))
        print('  ' + row)

    # 3) 射线自由性
    try:
        tf = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf)
        sc = st['scan']
        t = tf.lookup_transform('map', sc.header.frame_id, rospy.Time(0), rospy.Duration(2.0))
    except Exception as e:  # noqa
        print('TF failed: %s' % e)
        return 1
    tx, ty = t.transform.translation.x, t.transform.translation.y
    q = t.transform.rotation
    tyaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    ct, stt = math.cos(tyaw), math.sin(tyaw)

    # 分桶统计, 且把 occupied / unknown 分开:
    #   occupied = 地图真的胖 (看得穿的地方写了障碍)
    #   unknown  = 地图没建到 (不代表胖, 但规划器也会当不可走)
    # 采样用**固定绝对距离**, 且要求命中点至少再远 0.2m,
    # 否则 0.8*range 会落进"命中面 + 膨胀"里, 把正常墙算成地图偏胖。
    dists = [0.3, 0.5, 0.8, 1.2, 1.8]
    buckets = [('0.3m', 0.2, 0.4), ('0.5m', 0.4, 0.65), ('0.8m', 0.65, 1.0),
               ('1.2m', 1.0, 1.5), ('1.8m', 1.5, 2.5)]
    agg = {b[0]: {'n': 0, 'occ': 0, 'unk': 0, 'free': 0} for b in buckets}
    stats = {'free': 0, 'occ': 0, 'unk': 0, 'out': 0}
    ang = sc.angle_min
    for rng in sc.ranges:
        a = ang
        ang += sc.angle_increment
        if not (sc.range_min < rng < 6.0):
            continue
        for d in dists:
            if rng < d + 0.20:      # 命中点太近, 这个距离会落进命中面/膨胀
                continue
            lx, ly = d * math.cos(a), d * math.sin(a)
            wx = tx + ct * lx - stt * ly
            wy = ty + stt * lx + ct * ly
            v = cell(wx, wy)
            if v is None:
                stats['out'] += 1
                continue
            if v < 0:
                stats['unk'] += 1
                kind = 'unk'
            elif v >= 65:
                stats['occ'] += 1
                kind = 'occ'
            else:
                stats['free'] += 1
                kind = 'free'
            for name, lo, hi in buckets:
                if lo <= d < hi:
                    agg[name]['n'] += 1
                    agg[name][kind] += 1
                    break
    tot = float(sum(stats.values()) or 1)
    print('射线沿途采样: total=%d  free=%.1f%%  occupied=%.1f%%  unknown=%.1f%%  地图外=%.1f%%'
          % (sum(stats.values()), 100 * stats['free'] / tot, 100 * stats['occ'] / tot,
             100 * stats['unk'] / tot, 100 * stats['out'] / tot))
    print('按“距车体固定距离”分桶 (只算地图内采样):')
    print('  %-8s %6s %10s %10s %10s' % ('dist', 'n', 'occupied', 'unknown', 'free'))
    for name, lo, hi in buckets:
        b = agg[name]
        if b['n'] == 0:
            print('  %-8s %6d          -          -          -' % (name, 0))
            continue
        n = float(b['n'])
        print('  %-8s %6d %9.1f%% %9.1f%% %9.1f%%'
              % (name, b['n'], 100.0 * b['occ'] / n, 100.0 * b['unk'] / n, 100.0 * b['free'] / n))
    occ_pct = 100.0 * stats['occ'] / tot
    unk_pct = 100.0 * stats['unk'] / tot
    print('判定: occupied=%.1f%% (地图偏胖)  unknown=%.1f%% (地图没建到)' % (occ_pct, unk_pct))
    print('  occupied <=2%% 且 unknown <=5%% -> 地图正常;')
    print('  occupied >10%% -> 障碍被写进可见自由区(膨胀/高度带/动态物体);')
    print('  unknown 高 -> 建图覆盖不足, 需重录或补建。')
    return 0


if __name__ == '__main__':
    sys.exit(main())
