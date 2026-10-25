#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""定位拉偏恢复回归 (C2b/2D 约束).

用法 (已启动 sim_slam_nav.launch 并加载地图):
    rosrun rosiwit_slam_ros1 localization_kidnap_check.py

流程:
  1) 记录 TRACKING 位姿 P0
  2) /localization/set_initial_pose = P0 + (1.0, 0)   (人为拉偏 1m)
  3) 25s 内要求: health 离开 TRACKING 再回到 TRACKING, 且位姿回到 P0 附近
  4) 轨迹落盘 /tmp/kidnap_poses.csv (t,x,y,yaw), 供 render_route.py 出图
"""
from __future__ import print_function
import json
import math
import sys

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from xzrobot_ros_dependency.msg import LocalizationState
from xzrobot_ros_dependency.srv import Relocation, RelocationRequest

STATE = {"m": None}
TRACE = []
SEEN = []


def state_cb(msg):
    STATE["m"] = msg
    TRACE.append((rospy.get_time(), msg.pose.pose.position.x,
                  msg.pose.pose.position.y, 0.0))
    SEEN.append((rospy.get_time(), msg.health, msg.relocalize_count))


def dump(pull_xy, home_xy):
    with open("/tmp/kidnap_poses.csv", "w") as f:
        for (t, x, y, yaw) in TRACE:
            f.write("%.3f,%.4f,%.4f,%.4f\n" % (t, x, y, yaw))
    with open("/tmp/render_goals.txt", "w") as f:
        f.write("%.3f,%.3f,pull\n" % pull_xy)
        f.write("%.3f,%.3f,home\n" % home_xy)


def main():
    rospy.init_node("localization_kidnap_check", anonymous=True)
    rospy.Subscriber("/localization/state", LocalizationState, state_cb)
    fails = []

    msg = rospy.wait_for_message("/localization/state", LocalizationState, timeout=120)
    rospy.sleep(3.0)
    msg = STATE["m"]
    if msg.health != LocalizationState.HEALTH_TRACKING:
        print("not tracking at start (health=%d)" % msg.health)
        sys.exit(2)
    x0 = msg.pose.pose.position.x
    y0 = msg.pose.pose.position.y
    q0 = msg.pose.pose.orientation
    count0 = msg.relocalize_count
    print("0) locked at (%.3f, %.3f) matching=%.3f" % (x0, y0, msg.matching))

    req = RelocationRequest()
    req.map_name = msg.map_id
    req.initial_pose = PoseWithCovarianceStamped()
    req.initial_pose.header.frame_id = "map"
    req.initial_pose.pose.pose.position.x = x0 + 1.0
    req.initial_pose.pose.pose.position.y = y0
    req.initial_pose.pose.pose.orientation = q0
    rospy.wait_for_service("/localization/set_initial_pose")
    rospy.ServiceProxy("/localization/set_initial_pose", Relocation)(req)
    print("1) set_initial_pose -> P0 + (1.0, 0)")

    t0 = rospy.get_time()
    seen_bad = False
    recovered_at = None
    best = 1e9
    while rospy.get_time() - t0 < 30.0:
        m = STATE["m"]
        if m is not None:
            d = math.hypot(m.pose.pose.position.x - x0, m.pose.pose.position.y - y0)
            best = min(best, d)
            if m.health != LocalizationState.HEALTH_TRACKING:
                seen_bad = True
            if seen_bad and m.health == LocalizationState.HEALTH_TRACKING and d < 0.20:
                recovered_at = rospy.get_time() - t0
                break
        rospy.sleep(0.2)

    m = STATE["m"]
    d_final = math.hypot(m.pose.pose.position.x - x0, m.pose.pose.position.y - y0)
    print("2) health=%d dist_to_P0=%.3f best=%.3f reloc=%d->%d"
          % (m.health, d_final, best, count0, m.relocalize_count))
    if recovered_at is not None:
        print("   recovered at t=%.1fs" % recovered_at)
    dump((x0 + 1.0, y0), (x0, y0))

    if not seen_bad:
        fails.append("health never degraded after the pull")
    if m.health != LocalizationState.HEALTH_TRACKING:
        fails.append("health not TRACKING (%d)" % m.health)
    if d_final > 0.20:
        fails.append("pose not recovered (%.3f m)" % d_final)

    print("=== RESULT ===")
    if fails:
        print("FAIL: " + "; ".join(fails))
        sys.exit(1)
    print("PASS: kidnap recovery (2D constraint)")


if __name__ == "__main__":
    main()
