#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""判定"车身左前方点云"是自身部件还是环境: 必须让车动起来再看 (车上只读).

原理:
  - 自身部件 -> 在 base_link 下位置固定, 车动/转时一直出现在同一格;
  - 环境物体 -> 车一动, 它在 base_link 下的位置就变了, 不会长期占同一格.

用法:
    python2 cloud_self_motion_probe.py [秒数]   # 默认 30s, 期间请前进 1-2m + 原地转 >=30 度

输出:
  - 运动量 (位移/转角); 太小会提示无法判定
  - base_link 0.15m 栅格里"命中率 >=60% 帧"的格子 (这些才是跟车走的点)
  - 0.15m ASCII 命中率图 (0-9, A=100%)
"""
from __future__ import print_function
import math
import sys
import time
import collections

import rospy
import tf2_ros
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2

RES = 0.15
XMIN, XMAX, YMIN, YMAX = -1.2, 1.5, -1.2, 1.2
ZMIN, ZMAX = 0.10, 1.60


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def main():
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
    rospy.init_node('cloud_self_motion_probe', anonymous=True)
    box = {'m': None}
    rospy.Subscriber('/vanjee_points722', PointCloud2,
                     lambda m: box.__setitem__('m', m), queue_size=1)
    tf = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf)

    W = int((XMAX - XMIN) / RES)
    H = int((YMAX - YMIN) / RES)
    hit = [[0] * W for _ in range(H)]
    frames = 0
    seen = set()
    ref = None
    moved = 0.0
    turned = 0.0
    t0 = time.time()
    last_stamp = None
    print('collecting %.0fs: 请前进 1-2m 并原地转 >=30 度 ...' % seconds)
    while time.time() - t0 < seconds:
        time.sleep(0.4)
        m = box['m']
        if m is None or m.header.stamp == last_stamp:
            continue
        last_stamp = m.header.stamp
        try:
            t = tf.lookup_transform('odom', 'base_link', rospy.Time(0), rospy.Duration(0.3))
            tl = tf.lookup_transform('base_link', m.header.frame_id, m.header.stamp,
                                     rospy.Duration(0.3))
        except Exception:
            try:
                t = tf.lookup_transform('odom', 'base_link', rospy.Time(0), rospy.Duration(0.3))
                tl = tf.lookup_transform('base_link', m.header.frame_id, rospy.Time(0),
                                         rospy.Duration(0.3))
            except Exception:
                continue
        px, py, pyaw = t.transform.translation.x, t.transform.translation.y, yaw_of(t.transform.rotation)
        if ref is None:
            ref = (px, py, pyaw)
        moved = max(moved, math.hypot(px - ref[0], py - ref[1]))
        dyaw = abs(math.atan2(math.sin(pyaw - ref[2]), math.cos(pyaw - ref[2])))
        turned = max(turned, math.degrees(dyaw))

        tr = tl.transform
        q = tr.rotation
        xx, yy, zz, ww = q.x, q.y, q.z, q.w
        r00 = 1 - 2 * (yy * yy + zz * zz); r01 = 2 * (xx * yy - zz * ww); r02 = 2 * (xx * zz + yy * ww)
        r10 = 2 * (xx * yy + zz * ww); r11 = 1 - 2 * (xx * xx + zz * zz); r12 = 2 * (yy * zz - xx * ww)
        r20 = 2 * (xx * zz - yy * ww); r21 = 2 * (yy * zz + xx * ww); r22 = 1 - 2 * (xx * xx + yy * yy)
        tx, ty, tz = tr.translation.x, tr.translation.y, tr.translation.z

        cells = collections.Counter()
        for p in pc2.read_points(m, field_names=('x', 'y', 'z'), skip_nans=True):
            x, y, z = p
            bx = r00 * x + r01 * y + r02 * z + tx
            by = r10 * x + r11 * y + r12 * z + ty
            bz = r20 * x + r21 * y + r22 * z + tz
            if not (ZMIN <= bz <= ZMAX):
                continue
            if not (XMIN <= bx < XMAX and YMIN <= by < YMAX):
                continue
            gx = int((bx - XMIN) / RES)
            gy = int((by - YMIN) / RES)
            cells[(gx, gy)] += 1
        frames += 1
        for k, c in cells.items():
            if c >= 2:
                hit[k[1]][k[0]] += 1

    print('frames=%d  max_move=%.2fm  max_turn=%.1fdeg' % (frames, moved, turned))
    if frames < 5:
        print('FAIL: 没有收到足够的点云帧')
        return 1
    if moved < 0.5 and turned < 20.0:
        print('WARN: 车几乎没动, 无法区分"自身部件"和"环境物体"; 请重跑并前进/转圈')
    chars = '0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ'
    print('--- base_link 0.15m 命中率 (>=60%% 帧; . = 0%%) ---')
    hot = []
    for gy in range(H - 1, -1, -1):
        by = YMIN + (gy + 0.5) * RES
        row = ''
        for gx in range(W):
            bx = XMIN + (gx + 0.5) * RES
            r = float(hit[gy][gx]) / frames
            if -0.262 <= bx <= 0.644 and -0.338 <= by <= 0.338:
                row += 'F'
                continue
            if r < 0.05:
                row += '.'
            else:
                row += chars[min(int(r * 10), 35)]
            if r >= 0.6:
                hot.append((r, bx, by))
        print('%5.2f %s' % (by, row))
    hot.sort(reverse=True)
    print('--- 跟车走的格子 (命中率>=60%%) 前 15 ---')
    for r, bx, by in hot[:15]:
        print('   (x=%.2f, y=%.2f)  %.0f%%' % (bx, by, r * 100))
    if not hot:
        print('   无 -> 左前方点云是环境物体, 不是自身部件')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
