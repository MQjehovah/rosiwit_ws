#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""覆盖清洁 (navi_cmd=20) 回归.

用法 (需已启动 sim_slam_nav.launch 并加载地图):
    rosrun rosiwit_navigation_ros1 coverage_check.py [x_min y_min x_max y_max]

判据:
  1) action 结果 tracking_path_result == 1
  2) execution_percent 达到 >= 95
  3) 机器人真的动了 (轨迹长度 > 2m), 且轨迹不越出多边形 + 0.6m
  4) 日志里不出现 FAILED_COVERAGE*

背景: 本轮把代价地图语义统一成 nav_msgs (-1/0..100), 覆盖模式用的 astar
规划器阈值 (254->100) 与 buildCoverageCostmap (255->100) 都改了,
但覆盖模式此前从未跑过 —— 这个脚本就是补这个验证缺口.
"""
from __future__ import print_function
import math
import sys

import actionlib
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D
from xzrobot_ros_dependency.msg import NaviCmdActionAction, NaviCmdActionGoal

TRAJ = []
FEEDBACK = {"percent": 0.0, "navi_state": 0, "clean_area": 0.0}


def odom_cb(msg):
    TRAJ.append((msg.pose.pose.position.x, msg.pose.pose.position.y))


def feedback_cb(msg):
    FEEDBACK["percent"] = msg.execution_percent
    FEEDBACK["navi_state"] = msg.navi_state
    FEEDBACK["clean_area"] = msg.clean_area


def inside(poly, x, y):
    n = len(poly)
    ans = False
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        if ((y1 > y) != (y2 > y)):
            xin = (x2 - x1) * (y - y1) / (y2 - y1 + 1e-12) + x1
            if x < xin:
                ans = not ans
    return ans


def dist_to_poly(poly, x, y):
    if inside(poly, x, y):
        return 0.0
    return min(math.hypot(x - px, y - py) for (px, py) in poly)


def main():
    rospy.init_node("coverage_check", anonymous=True)
    if len(sys.argv) >= 5:
        poly = [(float(sys.argv[1]), float(sys.argv[2])),
                (float(sys.argv[3]), float(sys.argv[2])),
                (float(sys.argv[3]), float(sys.argv[4])),
                (float(sys.argv[1]), float(sys.argv[4]))]
    else:
        poly = [(-0.8, -0.35), (1.2, -0.35), (1.2, 0.35), (-0.8, 0.35)]

    rospy.Subscriber("/lio_odom", Odometry, odom_cb)
    rospy.wait_for_message("/lio_odom", Odometry, timeout=120)
    rospy.sleep(1.0)
    start = TRAJ[-1]
    print("polygon=%s start=(%.2f,%.2f)" % (poly, start[0], start[1]))
    if not inside(poly, start[0], start[1]):
        print("FAIL: start pose not inside polygon (coverage would abort)")
        sys.exit(2)

    client = actionlib.SimpleActionClient("/RobotNaviCmdAction", NaviCmdActionAction)
    if not client.wait_for_server(rospy.Duration(20.0)):
        print("FAIL: action server /RobotNaviCmdAction not available")
        sys.exit(2)

    goal = NaviCmdActionGoal()
    goal.navi_cmd = 20
    goal.robot_vel = 0.30
    goal.tracking_path = [Pose2D(x, y, 0.0) for (x, y) in poly]
    client.send_goal(goal, feedback_cb=feedback_cb)
    if not client.wait_for_result(rospy.Duration(240.0)):
        client.cancel_goal()
        print("FAIL: coverage timeout (percent=%.1f)" % FEEDBACK["percent"])
        sys.exit(1)

    res = client.get_result()
    rate = FEEDBACK["percent"]      # actionProgress() 返回 0..1 的覆盖率
    print("result tracking_path_result=%d coverage_rate=%.1f%% clean_area=%.2f m2"
          % (res.tracking_path_result, rate * 100.0, FEEDBACK["clean_area"]))

    fails = []
    if res.tracking_path_result != 1:
        fails.append("tracking_path_result=%d (expect 1)" % res.tracking_path_result)
    if rate < 0.95:
        fails.append("coverage rate %.1f%% < 95%%" % (rate * 100.0))

    path_len = 0.0
    for i in range(1, len(TRAJ)):
        path_len += math.hypot(TRAJ[i][0] - TRAJ[i - 1][0], TRAJ[i][1] - TRAJ[i - 1][1])
    worst = 0.0
    for (x, y) in TRAJ:
        worst = max(worst, dist_to_poly(poly, x, y))
    print("trajectory: len=%.2fm samples=%d max_outside=%.2fm" % (path_len, len(TRAJ), worst))
    if path_len < 2.0:
        fails.append("robot barely moved (%.2fm)" % path_len)
    if worst > 0.6:
        fails.append("trajectory left polygon by %.2fm" % worst)

    print("=== RESULT ===")
    if fails:
        print("FAIL: " + "; ".join(fails))
        sys.exit(1)
    print("PASS: coverage cleaning")


if __name__ == "__main__":
    main()
