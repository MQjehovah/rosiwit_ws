#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""清理地图 PGM 的地面 unknown 噪点 (与核心 unknown_free 后处理同口径).

规则 (逐 unknown 8-连通域):
  - 面积 <= --max-cells (0.05m 下 30 格 = 0.075 m^2)
  - 不接触地图边界
  - 至少有一个 free 邻居
  - 若贴 occupied: 平均自由邻居数 >= 2 (否则是墙后薄阴影, 保留)

用法:
    python2 clean_map_unknown.py <map_dir 或 map.yaml> [--max-cells 30] [--no-reload]

注意: 重新加载用 map.yaml 路径调 /switch_map, 绕过"同名地图快速路径".
"""
from __future__ import print_function

import argparse
import os
import sys
import time

import cv2
import numpy as np

OCC, FREE, UNK = 0, 254, 128


def map_base(p):
    p = p.rstrip('/')
    for suf in ('.yaml', '.pgm', '.xzmap', '.pcd', '.jpg'):
        if p.endswith('/map' + suf):
            return p[:-len(suf)]        # keep the trailing '/map'
    return p + '/map'


def clean(base, max_cells):
    pgm = base + '.pgm'
    img = cv2.imread(pgm, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise SystemExit('cannot read ' + pgm)
    h, w = img.shape
    tri = np.full((h, w), UNK, dtype=np.uint8)
    tri[img <= 104] = OCC
    tri[img >= 131] = FREE

    unk = (tri == UNK).astype(np.uint8)
    n, labels, stats, _ = cv2.connectedComponentsWithStats(unk, 8)
    kernel = np.ones((3, 3), np.uint8)
    cleared_c = cleared_n = kept_strip = 0
    for i in range(1, n):
        area = int(stats[i, cv2.CC_STAT_AREA])
        if area > max_cells:
            continue
        x, y = int(stats[i, cv2.CC_STAT_LEFT]), int(stats[i, cv2.CC_STAT_TOP])
        ww, hh = int(stats[i, cv2.CC_STAT_WIDTH]), int(stats[i, cv2.CC_STAT_HEIGHT])
        if x == 0 or y == 0 or x + ww >= w or y + hh >= h:
            continue
        # bbox 外扩 1 格, 计算 8 邻域里 free / occupied 的接触面
        x0, y0, x1, y1 = x - 1, y - 1, x + ww + 1, y + hh + 1
        patch = tri[y0:y1, x0:x1]
        sub = (labels[y0:y1, x0:x1] == i).astype(np.uint8)
        nbr = cv2.dilate(sub, kernel).astype(bool) & (~sub.astype(bool))
        free_n = int(np.count_nonzero(nbr & (patch == FREE)))
        touches_occ = bool(np.any(nbr & (patch == OCC)))
        if free_n == 0:
            continue
        if touches_occ and (float(free_n) / max(1, area)) < 2.0:
            kept_strip += 1
            continue
        patch[sub.astype(bool)] = FREE
        tri[y0:y1, x0:x1] = patch
        cleared_c += area
        cleared_n += 1

    ts = time.strftime('%Y%m%d_%H%M%S')
    os.system('cp -a "%s" "%s.bak_unknown_%s"' % (pgm, pgm, ts))
    if not cv2.imwrite(pgm, tri):
        raise SystemExit('write failed')
    cv2.imwrite(base + '.jpg', tri)

    print('cleaned %s' % pgm)
    print('  unknown comps=%d, cleared comps=%d cells=%d, kept wall-strips=%d' % (
        n - 1, cleared_n, cleared_c, kept_strip))
    print('  after: unknown=%d free=%d occ=%d' % (
        int((tri == UNK).sum()), int((tri == FREE).sum()), int((tri == OCC).sum())))
    return True


def reload_map(base, name):
    import rospy
    import tf2_ros
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from xzrobot_ros_dependency.srv import SwitchMap, SwitchMapRequest

    rospy.init_node('clean_map_unknown_reload', anonymous=True)
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
    req.map_dir = base + '.yaml'
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
