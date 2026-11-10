#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""拆开 /global_costmap 在车体附近的代价构成: 99(内切膨胀) vs 100(真障碍) vs 1..98."""
from __future__ import print_function

import math
import sys
import time

import rospy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String

FP = [(-0.262, -0.338), (0.644, -0.338), (0.644, 0.338), (-0.262, 0.338)]


def main():
    rospy.init_node('costmap_breakdown', anonymous=True)
    st = {'gc': None, 'lm': None, 'diag': None}
    rospy.Subscriber('/global_costmap', OccupancyGrid, lambda m: st.__setitem__('gc', m), queue_size=1)
    rospy.Subscriber('/local_map', OccupancyGrid, lambda m: st.__setitem__('lm', m), queue_size=1)
    rospy.Subscriber('/nav_diagnostics', String, lambda m: st.__setitem__('diag', m), queue_size=5)
    t0 = time.time()
    while time.time() - t0 < 20 and (st['gc'] is None or st['diag'] is None):
        time.sleep(0.2)
    import json
    d = json.loads(st['diag'].data)
    p = d.get('odom_pose') or {}
    px, py = p.get('x', 0.0), p.get('y', 0.0)
    pyaw = math.radians(p.get('yaw', 0.0))
    print('pose (%.3f, %.3f, %.2f deg)' % (px, py, p.get('yaw', 0.0)))

    def report(name, msg):
        if msg is None:
            print('  %s: no data' % name)
            return
        info = msg.info
        res, ox, oy = info.resolution, info.origin.position.x, info.origin.position.y
        w, h, data = info.width, info.height, msg.data
        c, s = math.cos(pyaw), math.sin(pyaw)
        hist = {}
        inside = 0
        inside_vals = {}
        for gy in range(h):
            wy = oy + (gy + 0.5) * res
            if abs(wy - py) > 0.9:
                continue
            for gx in range(w):
                wx = ox + (gx + 0.5) * res
                if abs(wx - px) > 0.9:
                    continue
                v = data[gy * w + gx]
                hist[v] = hist.get(v, 0) + 1
                bx = c * (wx - px) + s * (wy - py)
                by = -s * (wx - px) + c * (wy - py)
                if (-0.262 <= bx <= 0.644) and (-0.338 <= by <= 0.338):
                    inside += 1
                    inside_vals[v] = inside_vals.get(v, 0) + 1
        top = sorted(hist.items(), key=lambda kv: -kv[1])[:6]
        print('  %-16s 1.8m 窗口值分布(前6): %s' % (name, top))
        print('  %-16s footprint 内格数=%d  值分布=%s' % ('', inside, sorted(inside_vals.items())))

    report('/global_costmap', st['gc'])
    report('/local_map', st['lm'])
    return 0


if __name__ == '__main__':
    sys.exit(main())
