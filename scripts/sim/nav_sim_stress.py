#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Two-leg sim stress test for the costmap-worker refactor.

Leg 1: plain 3 m goal (baseline).
Leg 2: drop a keepout polygon across the straight line to a 4-6 m goal, wait for
       the planner view rebuild, then navigate around it.
Records /cmd_vel, /local_map, /global_costmap, /nav_diagnostics for both legs.
"""
from __future__ import print_function

import json
import math
import sys
import time

import rospy
from geometry_msgs.msg import Point, PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from xzrobot_ros_dependency.msg import VirtualForm, VirtualObstacles

TERMINAL = ('SUCCEEDED', 'FAILED_PLAN', 'FAILED_POSE', 'FAILED_PATH_FOOTPRINT',
            'FAILED_SAFETY_STOP', 'FAILED_NO_MOTION', 'FAILED_LOC_LOST',
            'FAILED_TF', 'STOPPED', 'STOPPED_ODOM', 'CANCELED')


class Drv(object):
    def __init__(self):
        self.map_msg = None
        self.diag = None
        self.reset_window()
        rospy.Subscriber('/map', OccupancyGrid, self.on_map, queue_size=1)
        rospy.Subscriber('/nav_diagnostics', String, self.on_diag, queue_size=20)
        rospy.Subscriber('/cmd_vel', Twist, self.on_cmd, queue_size=50)
        rospy.Subscriber('/local_map', OccupancyGrid, self.on_lm, queue_size=5)
        rospy.Subscriber('/global_costmap', OccupancyGrid, self.on_gc, queue_size=5)
        self.goal_pub = rospy.Publisher('/goal_pose', PoseStamped, queue_size=1)
        self.keepout_pub = rospy.Publisher('/virtual_obstacles', VirtualObstacles, queue_size=1)

    def reset_window(self):
        self.cmd_n = 0
        self.cmd_nan = 0
        self.cmd_max_v = 0.0
        self.cmd_max_w = 0.0
        self.cmd_dv = 0.0
        self.cmd_dw = 0.0
        self.prev = None
        self.cmd_first = None
        self.cmd_last = None
        self.lm_n = 0
        self.lm_first = None
        self.lm_last = None
        self.gc_n = 0
        self.gc_first = None
        self.gc_last = None
        self.safety_min = 1.0
        self.replan_fail = 0
        self.replan_count = 0
        self.plan_ms = -1.0
        self.states = []

    def on_map(self, msg):
        if msg.info.width > 0 and len(msg.data) > 0:
            self.map_msg = msg

    def on_diag(self, msg):
        try:
            d = json.loads(msg.data)
        except ValueError:
            return
        self.diag = d
        st = d.get('state')
        if st not in self.states:
            self.states.append(st)
        try:
            self.safety_min = min(self.safety_min, float(d.get('safety_scale', 1.0)))
        except (TypeError, ValueError):
            pass
        self.replan_fail = max(self.replan_fail, int(d.get('replan_fail', 0)))
        self.replan_count = max(self.replan_count, int(d.get('replan_count', 0)))
        self.plan_ms = max(self.plan_ms, float(d.get('plan_ms', -1)))

    def on_cmd(self, msg):
        v, w = msg.linear.x, msg.angular.z
        if any(map(lambda x: math.isnan(x) or math.isinf(x), (v, w))):
            self.cmd_nan += 1
            return
        if self.cmd_first is None:
            self.cmd_first = rospy.Time.now()
        self.cmd_last = rospy.Time.now()
        self.cmd_n += 1
        self.cmd_max_v = max(self.cmd_max_v, abs(v))
        self.cmd_max_w = max(self.cmd_max_w, abs(w))
        if self.prev is not None:
            self.cmd_dv = max(self.cmd_dv, abs(v - self.prev[0]))
            self.cmd_dw = max(self.cmd_dw, abs(w - self.prev[1]))
        self.prev = (v, w)

    def on_lm(self, msg):
        self.lm_n += 1
        if self.lm_first is None:
            self.lm_first = rospy.Time.now()
        self.lm_last = rospy.Time.now()

    def on_gc(self, msg):
        self.gc_n += 1
        if self.gc_first is None:
            self.gc_first = rospy.Time.now()
        self.gc_last = rospy.Time.now()

    # ---- map geometry ----
    def to_cell(self, x, y):
        i = self.map_msg.info
        return (int((x - i.origin.position.x) / i.resolution),
                int((y - i.origin.position.y) / i.resolution))

    def free(self, cx, cy):
        i = self.map_msg.info
        if cx < 0 or cy < 0 or cx >= i.width or cy >= i.height:
            return False
        return self.map_msg.data[cy * i.width + cx] == 0

    def clearance_ok(self, cx, cy, radius_m):
        res = self.map_msg.info.resolution
        r = int(math.ceil(radius_m / res))
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if dx * dx + dy * dy > r * r:
                    continue
                if not self.free(cx + dx, cy + dy):
                    return False
        return True

    def reach(self, start_cell, max_len_m):
        import heapq
        res = self.map_msg.info.resolution
        dist = {start_cell: 0.0}
        pq = [(0.0, start_cell)]
        nbrs = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
                (1, 1, 1.4142), (1, -1, 1.4142), (-1, 1, 1.4142), (-1, -1, 1.4142)]
        while pq:
            d, c = heapq.heappop(pq)
            if d > dist.get(c, 1e9) + 1e-9 or d > max_len_m:
                continue
            for dx, dy, k in nbrs:
                nb = (c[0] + dx, c[1] + dy)
                if not self.free(nb[0], nb[1]):
                    continue
                nd = d + k * res
                if nd < dist.get(nb, 1e9):
                    dist[nb] = nd
                    heapq.heappush(pq, (nd, nb))
        return dist

    def pick(self, pose, lo, hi, clear, mid=None):
        if mid is None:
            mid = 0.5 * (lo + hi)
        start = self.to_cell(pose[0], pose[1])
        if not self.free(*start):
            return None
        dist = self.reach(start, hi)
        best = None
        for cell, d in dist.items():
            if d < lo or d > hi:
                continue
            if not self.clearance_ok(cell[0], cell[1], clear):
                continue
            s = -abs(d - mid)
            if best is None or s > best[0]:
                best = (s, cell, d)
        if best is None:
            return None
        i = self.map_msg.info
        gx = i.origin.position.x + (best[1][0] + 0.5) * i.resolution
        gy = i.origin.position.y + (best[1][1] + 0.5) * i.resolution
        return gx, gy, best[2]

    @staticmethod
    def pose_of(d):
        p = d.get('odom_pose') or {}
        return (p.get('x', 0.0), p.get('y', 0.0), p.get('yaw', 0.0))

    def send_goal(self, gx, gy):
        yaw = math.atan2(gy - self.pose_of(self.diag)[1], gx - self.pose_of(self.diag)[0])
        m = PoseStamped()
        m.header.frame_id = 'map'
        m.pose.position.x = gx
        m.pose.position.y = gy
        m.pose.orientation.z = math.sin(yaw * 0.5)
        m.pose.orientation.w = math.cos(yaw * 0.5)
        # 真实使用(RViz/App)是单次发布; 连续重复发同一个 goal 会触发既有
        # "plan 在飞 + 重启"路径, 那是另一个问题, 这里不做重复发布.
        m.header.stamp = rospy.Time.now()
        self.goal_pub.publish(m)
        time.sleep(0.3)
        return yaw

    def wait_started(self, timeout=15.0):
        """等 goal_status 离开上一次的终态 (SUCCEEDED/FAILED_*/...)."""
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self.diag.get('goal_status') not in TERMINAL:
                return True
            time.sleep(0.05)
        return False

    def wait_done(self, timeout, retry_after=6.0, goal=None):
        t0 = time.time()
        resent = False
        while time.time() - t0 < timeout:
            gs = self.diag.get('goal_status')
            if gs in TERMINAL:
                return gs, resent
            if (not resent and goal is not None and time.time() - t0 > retry_after
                    and gs == 'ACCEPTED' and self.diag.get('state') == 'IDLE'):
                # goal 被接受但没起步(plan 结果被丢弃): 补发一次
                rospy.logwarn('goal accepted but idle for %.0fs; resending once', retry_after)
                self.send_goal(goal[0], goal[1])
                resent = True
            time.sleep(0.1)
        return 'TIMEOUT', resent

    def leg_report(self, label, goal, status, yaw):
        p = self.pose_of(self.diag)
        xy = math.hypot(p[0] - goal[0], p[1] - goal[1])
        dyaw = math.atan2(math.sin(math.radians(p[2]) - yaw),
                          math.cos(math.radians(p[2]) - yaw))
        dur = (self.cmd_last - self.cmd_first).to_sec() if (self.cmd_first and self.cmd_last) else 0.0
        lm = (self.lm_n - 1) / (self.lm_last - self.lm_first).to_sec() if (self.lm_first and self.lm_last) else 0.0
        gc = (self.gc_n - 1) / (self.gc_last - self.gc_first).to_sec() if (self.gc_first and self.gc_last) else 0.0
        return {
            'leg': label,
            'status': status,
            'states': self.states,
            'goal': [round(goal[0], 3), round(goal[1], 3)],
            'end_pose': [round(v, 3) for v in p],
            'xy_err_m': round(xy, 3),
            'yaw_err_rad': round(dyaw, 3),
            'cmd_n': self.cmd_n,
            'cmd_hz': round(self.cmd_n / dur, 2) if dur > 0 else 0.0,
            'cmd_nan': self.cmd_nan,
            'cmd_max_v': round(self.cmd_max_v, 3),
            'cmd_max_w': round(self.cmd_max_w, 3),
            'cmd_max_dv': round(self.cmd_dv, 3),
            'cmd_max_dw': round(self.cmd_dw, 3),
            'local_map_hz': round(lm, 2),
            'global_costmap_hz': round(gc, 2),
            'safety_scale_min': round(self.safety_min, 3),
            'replan_count': self.replan_count,
            'replan_fail': self.replan_fail,
            'plan_ms_max': round(self.plan_ms, 1),
        }


def main():
    label = sys.argv[1] if len(sys.argv) > 1 else 'stress'
    rospy.init_node('nav_sim_stress', anonymous=True)
    d = Drv()
    t0 = time.time()
    while (d.map_msg is None or d.diag is None) and time.time() - t0 < 60:
        time.sleep(0.2)
    if d.map_msg is None or d.diag is None:
        print(json.dumps({'label': label, 'error': 'no map/diag'}))
        return 1

    out = {'label': label, 'map_wh': [d.map_msg.info.width, d.map_msg.info.height]}

    # ---- leg 1: plain 3 m goal ----
    pose = d.pose_of(d.diag)
    g1 = d.pick(pose, 2.5, 3.5, 0.45)
    if g1 is None:
        print(json.dumps({'label': label, 'error': 'no leg1 goal'}))
        return 1
    d.reset_window()
    yaw1 = d.send_goal(g1[0], g1[1])
    d.wait_started()
    st1, resent1 = d.wait_done(120, goal=g1)
    out['leg1'] = d.leg_report('leg1_plain', g1, st1, yaw1)
    out['leg1']['resent'] = resent1
    print(json.dumps({'label': label, 'leg1': out['leg1']}))
    sys.stdout.flush()

    # ---- leg 2: continuous keepout churn while navigating ----
    # 目的不是"绕障", 而是持续触发 planner view 重建 (layers_mu_/map_mu_ 的热路径),
    # 同时控制环/worker 正常跑. 多边形放在路径侧向 1.2 m, 不改变路线.
    pose = d.pose_of(d.diag)
    g2 = d.pick(pose, 2.5, 3.5, 0.40)
    if g2 is None:
        g2 = d.pick(pose, 2.0, 4.0, 0.30)
    if g2 is None:
        out['leg2'] = {'leg': 'leg2_churn', 'error': 'no leg2 goal'}
        print(json.dumps(out))
        sys.stdout.flush()
        return 0

    dux, duy = g2[0] - pose[0], g2[1] - pose[1]
    n = math.hypot(dux, duy) or 1.0
    ux, uy = dux / n, duy / n
    px, py = -uy, ux
    cx = pose[0] + ux * 1.0 + px * 2.5
    cy = pose[1] + uy * 1.0 + py * 2.5
    pts = [(cx + px * 0.30 + ux * 0.30, cy + py * 0.30 + uy * 0.30),
           (cx + px * 0.30 - ux * 0.30, cy + py * 0.30 - uy * 0.30),
           (cx - px * 0.30 - ux * 0.30, cy - py * 0.30 - uy * 0.30),
           (cx - px * 0.30 + ux * 0.30, cy - py * 0.30 + uy * 0.30)]
    ko = VirtualObstacles()
    f = VirtualForm()
    f.form = [Point(x=p[0], y=p[1], z=0.0) for p in pts]
    ko.list = [f]
    out['keepout_center'] = [round(cx, 2), round(cy, 2)]

    stop_churn = {'v': False}
    sent = {'n': 0}

    def churn():
        while not stop_churn['v']:
            d.keepout_pub.publish(ko)
            sent['n'] += 1
            time.sleep(0.5)

    try:
        import threading
        th = threading.Thread(target=churn)
        th.daemon = True
        th.start()
    except Exception:
        th = None

    d.reset_window()
    yaw2 = d.send_goal(g2[0], g2[1])
    d.wait_started()
    st2, resent2 = d.wait_done(120, goal=g2)
    stop_churn['v'] = True
    out['leg2'] = d.leg_report('leg2_keepout_churn', g2, st2, yaw2)
    out['leg2']['resent'] = resent2
    out['keepout_publishes'] = sent['n']
    out['keepout_seen'] = d.diag.get('keepout')
    out['wall_s'] = round(time.time() - t0, 1)
    print(json.dumps(out))
    sys.stdout.flush()
    return 0


if __name__ == '__main__':
    sys.exit(main())
