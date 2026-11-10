#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""常驻 /map 发布器 (替代未安装的 map_server): 读 PGM/YAML, latch 持续发布."""
import io
import sys
import time

import rospy
import yaml
from nav_msgs.msg import OccupancyGrid, MapMetaData


def main():
    pgm = sys.argv[1]
    meta = yaml.safe_load(io.open(sys.argv[2], 'r'))
    res = float(meta['resolution'])
    ox, oy = float(meta['origin'][0]), float(meta['origin'][1])
    negate = int(meta.get('negate', 0))
    occ_th = float(meta.get('occupied_thresh', 0.65))
    free_th = float(meta.get('free_thresh', 0.196))

    f = io.open(pgm, 'rb')
    assert f.readline().strip() == b'P5'
    line = f.readline()
    while line.startswith(b'#'):
        line = f.readline()
    w, h = [int(v) for v in line.split()]
    f.readline()          # maxval
    raw = bytearray(f.read())

    vals = []
    for row in range(h - 1, -1, -1):     # PGM 从上到下 -> OccupancyGrid 从下到上
        for col in range(w):
            v = raw[row * w + col]
            occ = (v / 255.0) if negate else ((255.0 - v) / 255.0)
            if occ > occ_th:
                vals.append(100)
            elif occ < free_th:
                vals.append(0)
            else:
                vals.append(-1)

    grid = OccupancyGrid()
    grid.header.frame_id = 'map'
    grid.info = MapMetaData()
    grid.info.resolution = res
    grid.info.width = w
    grid.info.height = h
    grid.info.origin.position.x = ox
    grid.info.origin.position.y = oy
    grid.info.origin.orientation.w = 1.0
    grid.data = vals

    rospy.init_node('my_map_server')
    pub = rospy.Publisher('/map', OccupancyGrid, queue_size=1, latch=True)
    rate = rospy.Rate(2.0)
    rospy.loginfo('my_map_server: %dx%d res=%.3f origin=(%.1f,%.1f)', w, h, res, ox, oy)
    while not rospy.is_shutdown():
        grid.header.stamp = rospy.Time.now()
        pub.publish(grid)
        rate.sleep()


if __name__ == '__main__':
    main()
