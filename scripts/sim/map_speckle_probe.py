#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""静态地图 / costmap 的小斑点统计: 找出导航时看到的噪点来自哪一层."""
from __future__ import print_function
import time

import rospy
import numpy as np
import cv2
from nav_msgs.msg import OccupancyGrid


def analyze(name, msg, level):
    if msg is None:
        print('%s: no data' % name)
        return
    info = msg.info
    w, h = info.width, info.height
    arr = np.array(msg.data, dtype=np.int16).reshape(h, w)
    mask = (arr >= 65) if level == 'map' else (arr == 100)
    mask = mask.astype(np.uint8)
    n, labels, stats, cents = cv2.connectedComponentsWithStats(mask, 4)
    sizes = sorted([int(stats[i, cv2.CC_STAT_AREA]) for i in range(1, n)], reverse=True)
    small = [i for i in range(1, n) if stats[i, cv2.CC_STAT_AREA] <= 3]
    print('%s %dx%d res=%.3f  occupied=%d  comps=%d  max=%d  small(<=3格)=%d' % (
        name, w, h, info.resolution, int(mask.sum()), n - 1, sizes[0] if sizes else 0, len(small)))
    if sizes:
        print('   连通域大小分布(前12): %s' % sizes[:12])
    ox, oy, res = info.origin.position.x, info.origin.position.y, info.resolution
    for i in small[:12]:
        cx, cy = cents[i][0], cents[i][1]
        print('   small comp cells=%d at (%.2f, %.2f)' % (
            int(stats[i, cv2.CC_STAT_AREA]), ox + (cx + 0.5) * res, oy + (cy + 0.5) * res))


def main():
    rospy.init_node('map_speckle_probe', anonymous=True)
    st = {'map': None, 'gc': None, 'lc': None}
    rospy.Subscriber('/map', OccupancyGrid, lambda m: st.__setitem__('map', m), queue_size=1)
    rospy.Subscriber('/global_costmap', OccupancyGrid, lambda m: st.__setitem__('gc', m), queue_size=1)
    rospy.Subscriber('/local_costmap', OccupancyGrid, lambda m: st.__setitem__('lc', m), queue_size=1)
    t0 = time.time()
    while time.time() - t0 < 20 and (st['map'] is None or st['gc'] is None):
        time.sleep(0.1)
    analyze('/map        ', st['map'], 'map')
    analyze('/global_cost', st['gc'], 'cost')
    analyze('/local_cost ', st['lc'], 'cost')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
