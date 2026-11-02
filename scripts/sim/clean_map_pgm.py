#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""清理地图 PGM: 规整为三值图(0/128/254) + 删除 <=N 格孤立障碍, 可选重新加载.

用法:
    python2 clean_map_pgm.py <map_dir 或 map.yaml> [--max-cells 30] [--no-reload]

背景:
- 部分地图 PGM 是灰阶插值图 (墙边有 1-7 / 246-255 过渡值), 而核心 loadGridPgm
  只把**精确 0** 当障碍 -> 墙被切碎、散点残留。这里按 yaml 阈值规整成 0/128/254。
- 删掉 <=N 格的孤立障碍块 (与建图 min_component_cells 同口径, 0.05m 下 30 格 = 0.075m^2)。
- 重新加载必须用 **map.yaml 路径** 调 /switch_map: 同目录 + xzmap mtime 未变时,
  handleSwitchMap 会走"同名快速路径"跳过重载 (只比较 xzmap, 不看 pgm)。
"""
from __future__ import print_function

import argparse
import os
import sys
import time

import cv2
import numpy as np


def map_base(p):
    for suf in ('/map.yaml', '/map.pgm', '/map.xzmap', '/map.pcd', '/map.jpg', '/map'):
        if p.endswith(suf):
            return p[:-len(suf)]
    return p.rstrip('/') + '/map'


def clean(base, max_cells):
    pgm = base + '.pgm'
    img = cv2.imread(pgm, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise SystemExit('cannot read ' + pgm)
    h, w = img.shape
    tri = np.full((h, w), 128, dtype=np.uint8)   # unknown
    tri[img <= 104] = 0                          # occupied (p >= 0.59)
    tri[img >= 131] = 254                        # free (p < 0.49)

    occ = (tri == 0).astype(np.uint8)
    n, labels, stats, _ = cv2.connectedComponentsWithStats(occ, 8)
    removed_c = removed_n = 0
    for i in range(1, n):
        area = int(stats[i, cv2.CC_STAT_AREA])
        if area > max_cells:
            continue
        x, y = int(stats[i, cv2.CC_STAT_LEFT]), int(stats[i, cv2.CC_STAT_TOP])
        ww, hh = int(stats[i, cv2.CC_STAT_WIDTH]), int(stats[i, cv2.CC_STAT_HEIGHT])
        x0, y0 = max(0, x - 3), max(0, y - 3)
        x1, y1 = min(w, x + ww + 3), min(h, y + hh + 3)
        patch = tri[y0:y1, x0:x1]
        free_n = int((patch == 254).sum())
        unk_n = int((patch == 128).sum())
        fill = 254 if (free_n >= 3 and free_n >= unk_n) else 128
        roi = labels[y0:y1, x0:x1]
        patch[roi == i] = fill
        tri[y0:y1, x0:x1] = patch
        removed_c += area
        removed_n += 1

    ts = time.strftime('%Y%m%d_%H%M%S')
    os.system('cp -a "%s" "%s.bak_clean_%s"' % (pgm, pgm, ts))
    if not cv2.imwrite(pgm, tri):
        raise SystemExit('write failed')
    cv2.imwrite(base + '.jpg', tri)

    occ2 = (tri == 0).astype(np.uint8)
    n2, _, stats2, _ = cv2.connectedComponentsWithStats(occ2, 8)
    small2 = sum(1 for i in range(1, n2) if stats2[i, cv2.CC_STAT_AREA] <= max_cells)
    print('cleaned %s' % pgm)
    print('  removed comps=%d cells=%d' % (removed_n, removed_c))
    print('  after: occupied=%d comps=%d small(<=%d)=%d' % (
        int(occ2.sum()), n2 - 1, max_cells, small2))
    return True


def reload_map(base, name):
    import rospy
    import tf2_ros
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from xzrobot_ros_dependency.srv import SwitchMap, SwitchMapRequest

    rospy.init_node('clean_map_reload', anonymous=True)
    tf = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf)
    pose = None
    try:
        t = tf.lookup_transform('map', 'base_link', rospy.Time(0), rospy.Duration(2.0))
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = rospy.Time.now()
        pose.pose.pose.position.x = t.transform.translation.x
        pose.pose.pose.position.y = t.transform.translation.y
        pose.pose.pose.orientation = t.transform.rotation
    except Exception:
        pass
    rospy.wait_for_service('/switch_map', timeout=10.0)
    req = SwitchMapRequest()
    req.map_name = name
    req.map_dir = base + '.yaml'   # 关键: 用 yaml 路径绕过同名地图快速路径
    if pose is not None:
        req.initial_pose = pose
    resp = rospy.ServiceProxy('/switch_map', SwitchMap)(req)
    print('switch_map success=%s' % resp.success)


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('map')
    ap.add_argument('--max-cells', type=int, default=30)
    ap.add_argument('--no-reload', action='store_true')
    a = ap.parse_args()
    base = map_base(a.map)
    clean(base, a.max_cells)
    if not a.no_reload:
        reload_map(base, os.path.basename(os.path.dirname(base)))
