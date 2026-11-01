#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""旁路录像: 把 /nav_diagnostics + /cmd_vel 采样成 CSV, 并在结束时保存 /global_path.

用途: 定位"门前为什么被 STOP" —— 需要 y 方向每一拍的:
  位姿 (x,y) / 安全状态 / safety_scale / 指令 (v,w) / 路径点数

用法: python2 trace_nav.py out.csv  [每秒采样次数=20]
"""
from __future__ import print_function

import json
import math
import sys

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from std_msgs.msg import String


class Trace(object):
    def __init__(self, path):
        self.path_file = path + '.path'
        self.f = open(path, 'w')
        self.f.write('t,x,y,yaw,safety,goal_status,state,scale,v,w,path_pts,rec_stage,turn\n')
        self.t0 = None
        self.diag = None
        self.cmd = (0.0, 0.0)
        self.last_path = None
        self.path_saved = False
        self.odom = None
        rospy.Subscriber('/nav_diagnostics', String, self.on_diag, queue_size=20)
        rospy.Subscriber('/cmd_vel', Twist, self.on_cmd, queue_size=50)
        rospy.Subscriber('/global_path', Path, self.on_path, queue_size=5)
        # /nav_diagnostics 只有 1Hz, 看不清门前 1m 内的动态; 位姿改用 /odom (30Hz)
        rospy.Subscriber('/odom', Odometry, self.on_odom, queue_size=50)

    def on_odom(self, msg):
        p = msg.pose.pose
        q = p.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.odom = (p.position.x, p.position.y, yaw)

    def on_diag(self, msg):
        try:
            self.diag = json.loads(msg.data)
        except ValueError:
            return

    def on_cmd(self, msg):
        self.cmd = (msg.linear.x, msg.angular.z)

    def on_path(self, msg):
        if msg.poses:
            self.last_path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
            # 只保存**第一条**路径 (recovery 之前的规划结果), 用来判断
            # "规划出来的路径在门前是不是直线"
            if self.path_saved:
                return
            self.path_saved = True
            with open(self.path_file, 'w') as pf:
                pf.write('%.4f %.4f\n' % (rospy.Time.now().to_sec(), len(self.last_path)))
                for x, y in self.last_path:
                    pf.write('%.4f %.4f\n' % (x, y))

    def tick(self):
        d = self.diag
        if d is None:
            return
        now = rospy.Time.now().to_sec()
        if self.t0 is None:
            self.t0 = now
        p = d.get('odom_pose') or {}
        x = p.get('x', 0.0)
        y = p.get('y', 0.0)
        yaw_deg = p.get('yaw', 0.0)
        if self.odom is not None:
            x, y, yaw = self.odom
            yaw_deg = yaw * 180.0 / math.pi
        self.f.write('%7.2f,%.4f,%.4f,%.2f,%s,%s,%s,%.3f,%.4f,%.4f,%d,%d,%.1f\n' % (
            now - self.t0, x, y, yaw_deg,
            d.get('safety', '?'), d.get('goal_status', '?'),
            d.get('state', '?'), float(d.get('safety_scale', 1.0)),
            self.cmd[0], self.cmd[1], int(d.get('path_points', 0)),
            int(d.get('recovery_stage', 0)), float(d.get('path_turn_deg', 0.0))))
        self.f.flush()

    def close(self):
        self.f.close()
        if self.last_path:
            print('last_path_pts=%d' % len(self.last_path))
            print('last_path=' + ';'.join('%.3f,%.3f' % (x, y) for x, y in self.last_path))


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else 'trace.csv'
    hz = float(sys.argv[2]) if len(sys.argv) > 2 else 20.0
    rospy.init_node('nav_trace', anonymous=True)
    tr = Trace(out)
    r = rospy.Rate(hz)
    while not rospy.is_shutdown():
        tr.tick()
        try:
            r.sleep()
        except rospy.ROSTimeMovedBackwardsException:
            pass
    tr.close()


if __name__ == '__main__':
    main()
