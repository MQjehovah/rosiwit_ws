#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""组合录包: 停靠位录包循环 N 遍 (模拟长时间停放) 后接行驶录包 (模拟出库), 时间重排成连续的。
    python2 make_undock_bag.py <dock.bag> <route.bag> <loops> <out.bag>
两段之间停 0.5s。行驶段的 7.3m 位置跳变靠节点自己的重定位接上 (初始位姿还是停靠位的)。
"""
from __future__ import print_function
import sys

import rosbag
import rospy

dock_path, route_path, loops, out_path = sys.argv[1], sys.argv[2], int(sys.argv[3]), sys.argv[4]
LIDAR = ['/vanjee_points722', '/vanjee_lidar_imu_packets']
T0 = 1000.0


def rospy_time(s):
    return rospy.Time.from_sec(s)


def restamp(msg, t):
    # 消息头里的时间戳也要跟着改: 节点按 /clock (= 录包记录时间) 运行, 头里的时间还是原值的话
    # IMU/里程计缓冲、去畸变、TF 查询全都对不上 (2026-11-27 第一次做这个包就栽在这里)。
    if hasattr(msg, 'header'):
        msg.header.stamp = rospy.Time.from_sec(t)

dock = [(topic, msg, t.to_sec()) for topic, msg, t in rosbag.Bag(dock_path).read_messages(topics=LIDAR)]
d_t0, d_t1 = dock[0][2], dock[-1][2]
span = d_t1 - d_t0 + 0.1
print('停靠段: %.1fs x %d 遍' % (span, loops))

route_start = T0 + span * loops + 0.5
out = rosbag.Bag(out_path, 'w')
n = 0
for k in range(loops):
    off = T0 + span * k - d_t0
    for topic, msg, t in dock:
        restamp(msg, t + off)
        out.write(topic, msg, rospy_time(t + off))
        n += 1
r_t0 = None
m = 0
for topic, msg, t in rosbag.Bag(route_path).read_messages(topics=LIDAR + ['/odom']):
    ts = t.to_sec()
    if r_t0 is None:
        r_t0 = ts
    restamp(msg, ts - r_t0 + route_start)
    out.write(topic, msg, rospy_time(ts - r_t0 + route_start))
    m += 1
out.close()
print('写完: 停靠段 %d 条, 行驶段 %d 条 (行驶段从 t=%.1fs 开始)' % (n, m, route_start - T0))
