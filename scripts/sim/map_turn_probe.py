#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""建图"转弯黑块/前左鬼影"判别探针 (车上运行, 只读).

用法:
    python2 map_turn_probe.py [秒数]      # 默认 30 秒; 期间请建图并转弯

打印:
  1) 车体近处 occupied / unknown 的占比 (区分"真障碍" vs "未建到")
  2) 最大黑块在 base_link 下的位置/尺寸 (看是否固定在前左)
  3) 黑块是否有当前 /scan 回波支撑 (无支撑 = 鬼影/拖影)
  4) 跨帧统计: 同一 base_link 格是否总是黑 (跟着车 = 自身/拖影; 固定在地图 = 真实变化)
"""
from __future__ import print_function

import math
import sys
import time

import rospy
import tf2_ros
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan

FRONT_LEFT = (0.0, 3.0, 0.0, 3.0)   # bx_min, bx_max, by_min, by_max


def yaw_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def transform_xy(tf, target, source, stamp):
    try:
        t = tf.lookup_transform(target, source, stamp, rospy.Duration(0.2))
    except Exception:
        t = tf.lookup_transform(target, source, rospy.Time(0), rospy.Duration(1.0))
    tr = t.transform
    return tr.translation.x, tr.translation.y, yaw_quat(tr.rotation)


def main():
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
    rospy.init_node('map_turn_probe', anonymous=True)
    st = {'map': None, 'scan': None}
    rospy.Subscriber('/map', OccupancyGrid, lambda m: st.__setitem__('map', m), queue_size=1)
    rospy.Subscriber('/scan', LaserScan, lambda m: st.__setitem__('scan', m), queue_size=3)
    tf = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf)

    print('collecting %.0fs ... (请在建图模式里走 + 转弯)' % seconds)
    t0 = time.time()
    frames = 0
    skip = 0
    # base_link 0.1m 栅格: key -> 被占帧数
    persist = {}
    frame_cnt = 0
    # 汇总
    occ_total = 0
    unk_total = 0
    fl_occ = 0
    fl_unk = 0
    supported = 0
    unsupported = 0
    biggest = (0, 0.0, 0.0, 0.0, 0.0)   # area, cx, cy, w, h
    observed_pose = None

    while time.time() - t0 < seconds:
        time.sleep(1.0)
        m = st['map']
        sc = st['scan']
        if m is None:
            continue
        try:
            px, py, pyaw = transform_xy(tf, 'map', 'base_link', rospy.Time(0))
        except Exception:
            continue
        observed_pose = (px, py, pyaw)
        frame_cnt += 1
        info = m.info
        res, ox, oy = info.resolution, info.origin.position.x, info.origin.position.y
        w, h, data = info.width, info.height, m.data
        c, s = math.cos(pyaw), math.sin(pyaw)

        # /scan 命中点 -> map 格集合 (用于"黑块是否有回波支撑")
        scan_cells = set()
        scan_ok = False
        if sc is not None:
            try:
                sx, sy, syaw = transform_xy(tf, 'map', sc.header.frame_id, sc.header.stamp)
                cs, ss = math.cos(syaw), math.sin(syaw)
                a = sc.angle_min
                for r in sc.ranges:
                    ang = a
                    a += sc.angle_increment
                    if not (sc.range_min < r < sc.range_max):
                        continue
                    lx, ly = r * math.cos(ang), r * math.sin(ang)
                    wx = sx + cs * lx - ss * ly
                    wy = sy + ss * lx + cs * ly
                    scan_cells.add((int((wx - ox) / res), int((wy - oy) / res)))
                scan_ok = True
            except Exception:
                scan_ok = False

        # 车体 3m 内的 occupied / unknown + base 系栅格
        base_occ = {}
        for gy in range(h):
            wy = oy + (gy + 0.5) * res
            if abs(wy - py) > 3.0:
                continue
            row = gy * w
            for gx in range(w):
                v = data[row + gx]
                if v < 0:
                    wx = ox + (gx + 0.5) * res
                    if abs(wx - px) <= 3.0:
                        unk_total += 1
                        bx = c * (wx - px) + s * (wy - py)
                        by = -s * (wx - px) + c * (wy - py)
                        if FRONT_LEFT[0] <= bx <= FRONT_LEFT[1] and FRONT_LEFT[2] <= by <= FRONT_LEFT[3]:
                            fl_unk += 1
                    continue
                if v < 65:
                    continue
                wx = ox + (gx + 0.5) * res
                if abs(wx - px) > 3.0:
                    continue
                occ_total += 1
                bx = c * (wx - px) + s * (wy - py)
                by = -s * (wx - px) + c * (wy - py)
                if FRONT_LEFT[0] <= bx <= FRONT_LEFT[1] and FRONT_LEFT[2] <= by <= FRONT_LEFT[3]:
                    fl_occ += 1
                key = (int(math.floor(bx / 0.1)), int(math.floor(by / 0.1)))
                base_occ[key] = v
                # support
                if scan_ok:
                    hit = False
                    for ddx in (-2, -1, 0, 1, 2):
                        for ddy in (-2, -1, 0, 1, 2):
                            if (gx + ddx, gy + ddy) in scan_cells:
                                hit = True
                                break
                        if hit:
                            break
                    if hit:
                        supported += 1
                    else:
                        unsupported += 1
        # 最大连通域 (base 0.1m 栅格), 简单 4-邻域 flood fill
        seen = set()
        for k in base_occ:
            if k in seen:
                continue
            stack = [k]
            seen.add(k)
            cells = []
            while stack:
                cur = stack.pop()
                cells.append(cur)
                for nb in ((cur[0] + 1, cur[1]), (cur[0] - 1, cur[1]),
                           (cur[0], cur[1] + 1), (cur[0], cur[1] - 1)):
                    if nb in base_occ and nb not in seen:
                        seen.add(nb)
                        stack.append(nb)
            if len(cells) > biggest[0]:
                xs = [p[0] * 0.1 for p in cells]
                ys = [p[1] * 0.1 for p in cells]
                biggest = (len(cells), sum(xs) / len(xs), sum(ys) / len(ys),
                           max(xs) - min(xs) + 0.1, max(ys) - min(ys) + 0.1)
        for k in base_occ:
            persist[k] = persist.get(k, 0) + 1

    print('--- summary (%d frames, pose=%s) ---' % (
        frame_cnt, 'None' if observed_pose is None else '(%.2f, %.2f, %.0fdeg)' %
        (observed_pose[0], observed_pose[1], math.degrees(observed_pose[2]))))
    if frame_cnt == 0:
        print('no /map frames; 确认当前是建图模式且 /map 有订阅者')
        return 1
    tot = float(occ_total + unk_total or 1)
    print('车体 3m 内: occupied=%d (%.1f%%)  unknown=%d (%.1f%%)' % (
        occ_total, 100.0 * occ_total / tot, unk_total, 100.0 * unk_total / tot))
    print('前左象限(0<x<3, 0<y<3): occupied=%d  unknown=%d' % (fl_occ, fl_unk))
    print('最大黑块(base_link): cells=%d centroid=(%.2f, %.2f) extent=%.2fx%.2fm' % biggest)
    if supported + unsupported > 0:
        print('黑块有 /scan 回波支撑: %.1f%%  (无支撑=鬼影/拖影)' % (
            100.0 * supported / (supported + unsupported)))
    # 跨帧持久性
    hot = [(k, n) for k, n in persist.items() if n >= 0.6 * frame_cnt]
    hot.sort(key=lambda kv: -kv[1])
    print('base_link 下"总是黑"的 0.1m 格: %d 个 (>=60%% 帧)' % len(hot))
    for k, n in hot[:12]:
        print('   (%.1f, %.1f) %d/%d 帧' % (k[0] * 0.1, k[1] * 0.1, n, frame_cnt))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
