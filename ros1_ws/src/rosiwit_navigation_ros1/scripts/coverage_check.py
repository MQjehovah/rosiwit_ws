#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""覆盖清洁回归 + 数据落盘 (供 render_coverage.py 出图).

落盘 (默认 /tmp/cov):
    meta.json        地图几何 + 覆盖统计
    map.bin          原始 /map 数据 (int8, 行优先, 原点在左下)
    clean.bin        覆盖状态 (/clean_map, int8, 0=未覆盖 50=已覆盖 80=阻挡 100=障碍 -1=unknown/outside)
    traj.csv         机器人轨迹 t,x,y,yaw
    cmd.csv          t,linear.x,angular.z

用法:
    rosrun rosiwit_navigation_ros1 coverage_check.py -1.5 -0.3 1.5 2.2
"""
from __future__ import print_function
import json
import math
import os
import sys

import actionlib
import rospy
from geometry_msgs.msg import Pose2D, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from xzrobot_ros_dependency.msg import NaviCmdActionAction, NaviCmdActionGoal

OUT_DIR = "/tmp/cov"
STATE = {"map": None, "clean": None, "cmd": [], "traj": []}
FEEDBACK = {"percent": 0.0, "clean_area": 0.0}


def map_cb(msg):
    STATE["map"] = msg


def clean_cb(msg):
    STATE["clean"] = msg


def cmd_cb(msg):
    STATE["cmd"].append((rospy.get_time(), msg.linear.x, msg.angular.z))


def odom_cb(msg):
    q = msg.pose.pose.orientation
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                     1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    STATE["traj"].append((rospy.get_time(), msg.pose.pose.position.x,
                          msg.pose.pose.position.y, yaw))


def feedback_cb(msg):
    FEEDBACK["percent"] = msg.execution_percent
    FEEDBACK["clean_area"] = msg.clean_area


def send_goal(x, y):
    pub = rospy.Publisher("/goal_pose", PoseStamped, queue_size=1)
    rospy.sleep(0.3)
    g = PoseStamped()
    g.header.frame_id = "map"
    g.header.stamp = rospy.Time.now()
    g.pose.position.x = x
    g.pose.position.y = y
    g.pose.orientation.w = 1.0
    pub.publish(g)


def wait_status(timeout):
    t0 = rospy.get_time()
    s = ""
    while rospy.get_time() - t0 < timeout:
        s = STATE.get("status", "")
        if s:
            return s
        rospy.sleep(0.2)
    return "TIMEOUT"


def inside(poly, x, y):
    ans = False
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        if ((y1 > y) != (y2 > y)):
            if x < (x2 - x1) * (y - y1) / (y2 - y1 + 1e-12) + x1:
                ans = not ans
    return ans


def dump():
    if not os.path.isdir(OUT_DIR):
        os.makedirs(OUT_DIR)
    m = STATE["map"]
    c = STATE["clean"]
    if m is not None:
        with open(os.path.join(OUT_DIR, "map.bin"), "wb") as f:
            f.write(bytes(bytearray([v & 0xFF for v in m.data])))
    if c is not None:
        with open(os.path.join(OUT_DIR, "clean.bin"), "wb") as f:
            f.write(bytes(bytearray([v & 0xFF for v in c.data])))
    with open(os.path.join(OUT_DIR, "traj.csv"), "w") as f:
        for row in STATE["traj"]:
            f.write("%.3f,%.4f,%.4f,%.4f\n" % row)
    with open(os.path.join(OUT_DIR, "cmd.csv"), "w") as f:
        for row in STATE["cmd"]:
            f.write("%.3f,%.4f,%.4f\n" % row)
    meta = {
        "width": m.info.width if m else 0,
        "height": m.info.height if m else 0,
        "resolution": m.info.resolution if m else 0.0,
        "origin_x": m.info.origin.position.x if m else 0.0,
        "origin_y": m.info.origin.position.y if m else 0.0,
        "coverage_rate": FEEDBACK["percent"],
        "clean_area": FEEDBACK["clean_area"],
        "polygon": STATE.get("polygon", []),
    }
    with open(os.path.join(OUT_DIR, "meta.json"), "w") as f:
        json.dump(meta, f, indent=2)
    print("dumped to %s" % OUT_DIR)


def main():
    rospy.init_node("coverage_check", anonymous=True)
    if len(sys.argv) >= 5:
        poly = [(float(sys.argv[1]), float(sys.argv[2])),
                (float(sys.argv[3]), float(sys.argv[2])),
                (float(sys.argv[3]), float(sys.argv[4])),
                (float(sys.argv[1]), float(sys.argv[4]))]
    else:
        poly = [(-1.5, -0.3), (1.5, -0.3), (1.5, 2.2), (-1.5, 2.2)]
    STATE["polygon"] = poly

    rospy.Subscriber("/map", OccupancyGrid, map_cb)
    rospy.Subscriber("/clean_map", OccupancyGrid, clean_cb)
    rospy.Subscriber("/lio_odom", Odometry, odom_cb)
    from std_msgs.msg import String
    from geometry_msgs.msg import Twist
    rospy.Subscriber("/cmd_vel", Twist, cmd_cb)
    rospy.Subscriber("/goal_status", String,
                     lambda m: STATE.__setitem__("status", m.data))
    rospy.wait_for_message("/lio_odom", Odometry, timeout=120)
    rospy.sleep(1.0)
    start = STATE["traj"][-1]
    print("polygon=%s start=(%.2f,%.2f)" % (poly, start[1], start[2]))
    if not inside(poly, start[1], start[2]):
        print("FAIL: start pose not inside polygon")
        dump()
        sys.exit(2)

    client = actionlib.SimpleActionClient("/RobotNaviCmdAction", NaviCmdActionAction)
    if not client.wait_for_server(rospy.Duration(20.0)):
        print("FAIL: action server not available")
        dump()
        sys.exit(2)

    goal = NaviCmdActionGoal()
    goal.navi_cmd = 20
    goal.robot_vel = 0.30
    goal.tracking_path = [Pose2D(x, y, 0.0) for (x, y) in poly]
    client.send_goal(goal, feedback_cb=feedback_cb)
    if not client.wait_for_result(rospy.Duration(480.0)):
        client.cancel_goal()
        print("FAIL: coverage timeout (rate=%.1f%%)" % (FEEDBACK["percent"] * 100.0))
        rospy.sleep(1.0)
        dump()
        sys.exit(1)

    res = client.get_result()
    print("result tracking_path_result=%d coverage_rate=%.1f%% clean_area=%.2f m2"
          % (res.tracking_path_result, FEEDBACK["percent"] * 100.0,
             FEEDBACK["clean_area"]))
    rospy.sleep(1.0)   # 等最后一帧 /clean_map
    dump()

    fails = []
    if res.tracking_path_result != 1:
        fails.append("tracking_path_result=%d (expect 1)" % res.tracking_path_result)
    if FEEDBACK["percent"] < 0.95:
        fails.append("coverage rate %.1f%% < 95%%" % (FEEDBACK["percent"] * 100.0))

    path_len = 0.0
    for i in range(1, len(STATE["traj"])):
        path_len += math.hypot(STATE["traj"][i][1] - STATE["traj"][i - 1][1],
                               STATE["traj"][i][2] - STATE["traj"][i - 1][2])
    print("trajectory length=%.2f m samples=%d" % (path_len, len(STATE["traj"])))
    if path_len < 2.0:
        fails.append("robot barely moved (%.2fm)" % path_len)

    print("=== RESULT ===")
    if fails:
        print("FAIL: " + "; ".join(fails))
        sys.exit(1)
    print("PASS: coverage cleaning")


if __name__ == "__main__":
    main()
