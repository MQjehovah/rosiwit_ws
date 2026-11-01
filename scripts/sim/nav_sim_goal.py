#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Sim navigation A/B driver for rosiwit_navigation_ros1.

- waits for /map, picks a reachable free goal 2..4 m away (BFS on the map)
- publishes it to /goal_pose (native interface)
- records /nav_diagnostics, /cmd_vel, /local_map, /global_costmap
- prints one JSON line with the run summary
"""
from __future__ import print_function

import json
import math
import sys
import time

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import String


class Run(object):
    def __init__(self, timeout):
        self.timeout = timeout
        self.map_msg = None
        self.diag = None
        self.diag_hist = []
        self.cmd_count = 0
        self.cmd_nan = 0
        self.cmd_first = None
        self.cmd_last = None
        self.cmd_max_v = 0.0
        self.cmd_v_sum = 0.0
        self.cmd_max_w = 0.0
        self.cmd_dv = 0.0
        self.cmd_dw = 0.0
        self.prev_cmd = None
        self.local_map_count = 0
        self.global_costmap_count = 0
        self.lm_first = None
        self.lm_last = None
        self.gc_first = None
        self.gc_last = None
        self.path = None
        self.clearance_samples = []
        self.track_errs = []
        self.t_start = None

        rospy.Subscriber('/map', OccupancyGrid, self.on_map, queue_size=1)
        rospy.Subscriber('/nav_diagnostics', String, self.on_diag, queue_size=20)
        rospy.Subscriber('/cmd_vel', Twist, self.on_cmd, queue_size=50)
        rospy.Subscriber('/local_map', OccupancyGrid, self.on_local_map, queue_size=5)
        rospy.Subscriber('/global_costmap', OccupancyGrid, self.on_global, queue_size=5)
        rospy.Subscriber('/global_path', Path, self.on_path, queue_size=2)
        self.goal_pub = rospy.Publisher('/goal_pose', PoseStamped, queue_size=1)

    def nearest_lethal(self, pose):
        # distance from robot to nearest lethal cell (local window search)
        if self.map_msg is None:
            return None
        info = self.map_msg.info
        res = info.resolution
        cx, cy = self.to_cell(pose[0], pose[1])
        best = 1e9
        for dy in range(-20, 21):
            for dx in range(-20, 21):
                gx, gy = cx + dx, cy + dy
                if gx < 0 or gy < 0 or gx >= info.width or gy >= info.height:
                    continue
                if self.map_msg.data[gy * info.width + gx] >= 100:
                    d = math.hypot(dx * res, dy * res)
                    if d < best:
                        best = d
        return None if best > 1e9 else best

    def on_path(self, msg):
        if msg.poses:
            self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def track_err(self):
        """当前位姿到 /global_path 折线的最近距离 (m)."""
        if not self.path or self.diag is None:
            return None
        p = self.diag.get('odom_pose') or {}
        x, y = p.get('x'), p.get('y')
        if x is None or y is None:
            return None
        best = 1e9
        for i in range(len(self.path) - 1):
            ax, ay = self.path[i]
            bx, by = self.path[i + 1]
            dx, dy = bx - ax, by - ay
            L2 = dx * dx + dy * dy
            if L2 < 1e-12:
                d = math.hypot(x - ax, y - ay)
            else:
                t = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / L2))
                d = math.hypot(x - (ax + t * dx), y - (ay + t * dy))
            if d < best:
                best = d
        return best

    def on_map(self, msg):
        if msg.info.width > 0 and len(msg.data) > 0:
            self.map_msg = msg

    def on_diag(self, msg):
        try:
            d = json.loads(msg.data)
        except ValueError:
            return
        self.diag = d
        self.diag_hist.append(d)

    def on_cmd(self, msg):
        v = msg.linear.x
        w = msg.angular.z
        if math.isnan(v) or math.isnan(w) or math.isinf(v) or math.isinf(w):
            self.cmd_nan += 1
            return
        if self.cmd_first is None:
            self.cmd_first = rospy.Time.now()
        self.cmd_last = rospy.Time.now()
        self.cmd_count += 1
        self.cmd_max_v = max(self.cmd_max_v, abs(v))
        self.cmd_v_sum += abs(v)
        self.cmd_max_w = max(self.cmd_max_w, abs(w))
        if self.prev_cmd is not None:
            self.cmd_dv = max(self.cmd_dv, abs(v - self.prev_cmd[0]))
            self.cmd_dw = max(self.cmd_dw, abs(w - self.prev_cmd[1]))
        self.prev_cmd = (v, w)

    def on_local_map(self, msg):
        self.local_map_count += 1
        if self.lm_first is None:
            self.lm_first = rospy.Time.now()
        self.lm_last = rospy.Time.now()

    def on_global(self, msg):
        self.global_costmap_count += 1
        if self.gc_first is None:
            self.gc_first = rospy.Time.now()
        self.gc_last = rospy.Time.now()

    # ---------- map helpers ----------
    def to_cell(self, x, y):
        info = self.map_msg.info
        cx = int((x - info.origin.position.x) / info.resolution)
        cy = int((y - info.origin.position.y) / info.resolution)
        return cx, cy

    def free(self, cx, cy):
        info = self.map_msg.info
        if cx < 0 or cy < 0 or cx >= info.width or cy >= info.height:
            return False
        return self.map_msg.data[cy * info.width + cx] == 0

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

    def bfs(self, start, max_len_m):
        """Dijkstra-ish BFS with 8-connectivity, returns dict cell->dist_m."""
        info = self.map_msg.info
        res = info.resolution
        dist = {start: 0.0}
        frontier = [(0.0, start)]
        import heapq
        nbrs = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
                (1, 1, 1.4142), (1, -1, 1.4142), (-1, 1, 1.4142), (-1, -1, 1.4142)]
        while frontier:
            d, cell = heapq.heappop(frontier)
            if d > dist.get(cell, 1e9) + 1e-9:
                continue
            if d > max_len_m:
                continue
            for dx, dy, w in nbrs:
                nb = (cell[0] + dx, cell[1] + dy)
                if not self.free(nb[0], nb[1]):
                    continue
                nd = d + w * res
                if nd < dist.get(nb, 1e9):
                    dist[nb] = nd
                    heapq.heappush(frontier, (nd, nb))
        return dist

    def pick_goal(self, pose):
        start = self.to_cell(pose[0], pose[1])
        if not self.free(*start):
            return None
        band = getattr(self, 'goal_band', None)
        max_len = max(5.0, band[1] if band else 5.0)
        dist = self.bfs(start, max_len)
        bands = []
        if band:
            bands.append((band[0], band[1], 0.5 * (band[0] + band[1]), 0.35))
        bands += [(2.0, 4.0, 3.0, 0.45), (1.2, 5.0, 2.5, 0.35), (0.8, 5.0, 1.8, 0.25)]
        for lo, hi, mid, clear in bands:
            best = None
            for cell, d in dist.items():
                if d < lo or d > hi:
                    continue
                if not self.clearance_ok(cell[0], cell[1], clear):
                    continue
                score = -abs(d - mid)
                if best is None or score > best[0]:
                    best = (score, cell, d)
            if best is not None:
                info = self.map_msg.info
                gx = info.origin.position.x + (best[1][0] + 0.5) * info.resolution
                gy = info.origin.position.y + (best[1][1] + 0.5) * info.resolution
                yaw = math.atan2(gy - pose[1], gx - pose[0])
                return (gx, gy, yaw, best[2])
        return None


def pose_from_diag(d):
    p = d.get('odom_pose') or {}
    return (p.get('x', 0.0), p.get('y', 0.0), p.get('yaw', 0.0))


def main():
    label = sys.argv[1] if len(sys.argv) > 1 else 'run'
    timeout = float(sys.argv[2]) if len(sys.argv) > 2 else 150.0
    # 可选第 3 参数: 目标距离带 "lo,hi" (默认 2.0,4.0 -> 3.0)
    goal_band = None
    if len(sys.argv) > 3 and ',' in sys.argv[3]:
        a, b = sys.argv[3].split(',')
        goal_band = (float(a), float(b))
    # fixed goal: argv[4] = "x,y[,yaw]" (repeatable scenes)
    fixed_goal = None
    if len(sys.argv) > 4 and ',' in sys.argv[4]:
        parts = [float(v) for v in sys.argv[4].split(',')]
        if len(parts) >= 2:
            fixed_goal = parts
    rospy.init_node('nav_sim_driver', anonymous=True)
    run = Run(timeout)
    run.goal_band = goal_band

    t0 = time.time()
    while run.map_msg is None or run.diag is None:
        if time.time() - t0 > 60:
            print(json.dumps({'label': label, 'error': 'no map/diagnostics within 60s'}))
            return 1
        time.sleep(0.2)

    pose = pose_from_diag(run.diag)
    if fixed_goal is not None:
        gx0, gy0 = fixed_goal[0], fixed_goal[1]
        gyaw0 = (fixed_goal[2] if len(fixed_goal) > 2
                 else math.atan2(gy0 - pose[1], gx0 - pose[0]))
        goal = (gx0, gy0, gyaw0, math.hypot(gx0 - pose[0], gy0 - pose[1]))
    else:
        goal = run.pick_goal(pose)
    if goal is None:
        info = run.map_msg.info
        print(json.dumps({'label': label, 'error': 'no reachable goal',
                          'map': [info.width, info.height],
                          'pose': pose}))
        return 1

    gx, gy, gyaw, bfs_len = goal
    msg = PoseStamped()
    msg.header.frame_id = 'map'
    msg.header.stamp = rospy.Time.now()
    msg.pose.position.x = gx
    msg.pose.position.y = gy
    msg.pose.position.z = 0.0
    msg.pose.orientation.z = math.sin(gyaw * 0.5)
    msg.pose.orientation.w = math.cos(gyaw * 0.5)
    run.t_start = rospy.Time.now()
    msg.header.stamp = rospy.Time.now()
    run.goal_pub.publish(msg)
    time.sleep(0.3)

    terminal = ('SUCCEEDED', 'FAILED_PLAN', 'FAILED_POSE', 'FAILED_PATH_FOOTPRINT',
                'FAILED_SAFETY_STOP', 'FAILED_NO_MOTION', 'FAILED_LOC_LOST',
                'FAILED_TF', 'STOPPED', 'STOPPED_ODOM', 'CANCELED')
    seen_states = []
    safety_min = 1.0
    health_min = 1.0
    replan_fail_max = 0
    replan_count_max = 0
    plan_ms_max = -1.0
    status = 'TIMEOUT'
    resent = False
    while not rospy.is_shutdown():
        if time.time() - t0 > 60 + timeout:
            status = 'TIMEOUT'
            break
        d = run.diag
        if d is None:
            continue
        st = d.get('state')
        gs = d.get('goal_status')
        if st not in seen_states:
            seen_states.append(st)
        if st == 'NAVIGATING':
            _c = run.nearest_lethal(pose_from_diag(d))
            if _c is not None:
                run.clearance_samples.append(_c)
            _e = run.track_err()
            if _e is not None:
                run.track_errs.append(_e)
        try:
            safety_min = min(safety_min, float(d.get('safety_scale', 1.0)))
        except (TypeError, ValueError):
            pass
        try:
            health_min = min(health_min, float(d.get('health_scale', 1.0)))
        except (TypeError, ValueError):
            pass
        replan_fail_max = max(replan_fail_max, int(d.get('replan_fail', 0)))
        replan_count_max = max(replan_count_max, int(d.get('replan_count', 0)))
        plan_ms_max = max(plan_ms_max, float(d.get('plan_ms', -1)))
        # 终态判定用前缀: FAILED_* / STOPPED_* 都是终态
        # (之前穷举列表漏了 FAILED_RECOVERY, 导致 harness 白等 210s)
        if (gs == 'SUCCEEDED' or gs == 'CANCELED'
                or (gs and gs.startswith('FAILED'))
                or (gs and gs.startswith('STOPPED'))):
            status = gs
            break
        # 单次发布(RViz/App 语义); 若 ACCEPTED 但一直 IDLE, 补发一次
        if (not resent and gs == 'ACCEPTED' and st == 'IDLE'
                and time.time() - t0 > 66):
            msg.header.stamp = rospy.Time.now()
            run.goal_pub.publish(msg)
            resent = True
        time.sleep(0.1)

    p = pose_from_diag(run.diag)
    xy_err = math.hypot(p[0] - gx, p[1] - gy)
    # /nav_diagnostics 的 yaw 是**度** (日志可读性), 换算成弧度再比
    end_yaw_rad = math.radians(p[2])
    dyaw = math.atan2(math.sin(end_yaw_rad - gyaw), math.cos(end_yaw_rad - gyaw))
    dur = (run.cmd_last - run.cmd_first).to_sec() if (run.cmd_first and run.cmd_last) else 0.0
    cmd_hz = run.cmd_count / dur if dur > 0 else 0.0
    lm_dur = (run.lm_last - run.lm_first).to_sec() if (run.lm_first and run.lm_last) else 0.0
    gc_dur = (run.gc_last - run.gc_first).to_sec() if (run.gc_first and run.gc_last) else 0.0
    lm_hz = (run.local_map_count - 1) / lm_dur if lm_dur > 0 else 0.0
    gc_hz = (run.global_costmap_count - 1) / gc_dur if gc_dur > 0 else 0.0
    out = {
        'label': label,
        'status': status,
        'states_seen': seen_states,
        'min_clearance_m': (round(min(run.clearance_samples), 3)
                            if run.clearance_samples else None),
        'xtrack_rms_m': (round(math.sqrt(sum(e * e for e in run.track_errs) / len(run.track_errs)), 3)
                         if run.track_errs else None),
        'xtrack_max_m': round(max(run.track_errs), 3) if run.track_errs else None,
        'goal': [round(gx, 3), round(gy, 3), round(gyaw, 3)],
        'bfs_len_m': round(bfs_len, 2),
        'start_pose': [round(v, 3) for v in pose],
        'end_pose': [round(v, 3) for v in p],
        'xy_err_m': round(xy_err, 3),
        'yaw_err_rad': round(dyaw, 3),
        'cmd_n': run.cmd_count,
        'cmd_hz': round(cmd_hz, 2),
        'cmd_max_v': round(run.cmd_max_v, 3),
        'cmd_mean_v': round(run.cmd_v_sum / max(1, run.cmd_count), 3),
        'cmd_max_w': round(run.cmd_max_w, 3),
        'cmd_max_dv': round(run.cmd_dv, 3),
        'cmd_max_dw': round(run.cmd_dw, 3),
        'cmd_nan': run.cmd_nan,
        'local_map_hz': round(lm_hz, 2),
        'global_costmap_hz': round(gc_hz, 2),
        'safety_scale_min': round(safety_min, 3),
        'health_scale_min': round(health_min, 3),
        'replan_count': replan_count_max,
        'replan_fail': replan_fail_max,
        'plan_ms_max': round(plan_ms_max, 1),
        'goal_resent': resent,
        'wall_s': round(time.time() - t0, 1),
    }
    print(json.dumps(out))
    return 0


if __name__ == '__main__':
    sys.exit(main())
