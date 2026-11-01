#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""从几何参数生成 doorway 场景的占据栅格地图 (map.pgm + map.yaml).

为什么要有生成器: 原来 maps/doorway/map.pgm 是手工产物, 门洞宽度改了没法复现,
而且地图与 world 容易不一致 (栅格化会把 0.75m 门吃成 0.70m)。

地图内容 (与 world/doorway.world 同源):
  * 侧墙   x = +-1.5 (厚 0.2)          -> 起点走廊, 避免地图全 unknown
  * 门墙   y = 3.0  (厚 0.2), 门洞宽 gap_w, 中心 x=0
  * 后墙   y = 5.0  (厚 0.2)

用法: gen_doorway_map.py <out_dir> [gap_w=0.80] [res=0.05]
"""
from __future__ import print_function

import math
import os
import sys

FREE = 254
OCC = 0


def main():
    out = sys.argv[1]
    gap_w = float(sys.argv[2]) if len(sys.argv) > 2 else 0.80
    res = float(sys.argv[3]) if len(sys.argv) > 3 else 0.05
    # 覆盖 x -2..2, y -1..5.5
    ox, oy = -2.0, -1.0
    w = int(round((2.0 - ox) / res))       # 80
    h = int(round((5.5 - oy) / res))       # 130
    half = 0.5 * gap_w

    def occupied(wx, wy):
        # 侧墙
        if abs(wx) >= 1.4 and abs(wx) <= 1.6 and wy <= 3.0:
            return True
        # 门墙 (厚 0.2, 门洞 |x| <= half)
        if 2.9 <= wy <= 3.1 and abs(wx) > half:
            return True
        # 后墙
        if 4.9 <= wy <= 5.1:
            return True
        return False

    cells = []
    for r in range(h):
        wy = oy + (h - 1 - r + 0.5) * res        # PGM 行 0 = 图顶部 = y 最大
        row = []
        for c in range(w):
            wx = ox + (c + 0.5) * res
            row.append(OCC if occupied(wx, wy) else FREE)
        cells.append(row)

    pgm = os.path.join(out, 'map.pgm')
    with open(pgm, 'wb') as f:
        f.write(b'P5\n')
        f.write(('%d %d\n' % (w, h)).encode())
        f.write(b'255\n')
        f.write(bytes(bytearray([v for row in cells for v in row])))
    with open(os.path.join(out, 'map.yaml'), 'w') as f:
        f.write('image: map.pgm\n')
        f.write('resolution: %.2f\n' % res)
        f.write('origin: [%.4f, %.4f, 0.0]\n' % (ox, oy))
        f.write('negate: 0\n')
        f.write('occupied_thresh: 0.65\n')
        f.write('free_thresh: 0.196\n')
    print('wrote %s (%dx%d, gap=%.2fm, res=%.2f)' % (pgm, w, h, gap_w, res))


if __name__ == '__main__':
    main()
