#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""free+rpp 多方向(倒车)回归.

用法:
    roslaunch rosiwit_starter_ros1 sim_slam_nav.launch gui:=false rviz:=false \
        map:=$HOME/rosiwit_ws/docs/maps/house_sim
    rostopic pub -1 /set_motion_profile std_msgs/String "data: 'free,rpp'"
    rosrun rosiwit_navigation_ros1 free_rpp_check.py

场景:
  1) 前进到 A(1.5, 0) 并停稳 (车头朝 +x)
  2) 目标 B(0.3, 0): 在身后 1.2m —— **必须超过末段精调半径 0.6m**,
     否则会走节点的 precision-approach 分支(只前进), 测不到倒车
  3) 再前进回 A

判据: 第 2 段出现 cmd_vel.linear.x < -0.05 (真的倒车), 三段都到达.

设计边界 (2026-10-25 实测): 末段 0.6m 以内由节点 precision-approach 接管,
该分支只前进 —— 所以"距离 < 0.6m 的倒车目标"会表现为掉头前进, 不是 bug.
"""
from __future__ import print_function
import math
import sys

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

STATUS = {"s": ""}
CMD = []
ODOM = []


def status_cb(msg):
    STATUS["s"] = msg.data


def cmd_cb(msg):
    CMD.append((rospy.get_time(), msg.linear.x, msg.angular.z))


def odom_cb(msg):
    ODOM.append((rospy.get_time(), msg.pose.pose.position.x,
                 msg.pose.pose.position.y))


def send_goal(x, y, yaw_deg=0.0):
    pub = rospy.Publisher("/goal_pose", PoseStamped, queue_size=1)
    rospy.sleep(0.3)
    g = PoseStamped()
    g.header.frame_id = "map"
    g.header.stamp = rospy.Time.now()
    g.pose.position.x = x
    g.pose.position.y = y
    yaw = math.radians(yaw_deg)
    g.pose.orientation.z = math.sin(yaw / 2.0)
    g.pose.orientation.w = math.cos(yaw / 2.0)
    pub.publish(g)


def wait_done(timeout):
    t0 = rospy.get_time()
    while rospy.get_time() - t0 < timeout:
        s = STATUS["s"]
        if "SUCCEEDED" in s or "FAILED" in s or "ABORTED" in s:
            return s
        rospy.sleep(0.2)
    return "TIMEOUT:" + STATUS["s"]


def phase(name, gx, gy):
    STATUS["s"] = ""
    t0 = rospy.get_time()
    n0 = len(CMD)
    send_goal(gx, gy)
    result = wait_done(120)
    cmds = [c for c in CMD[n0:] if c[0] >= t0]
    vmin = min([c[1] for c in cmds]) if cmds else 0.0
    vmax = max([c[1] for c in cmds]) if cmds else 0.0
    rev = sum(1 for c in cmds if c[1] < -0.05)
    pose = ODOM[-1] if ODOM else (0.0, 0.0, 0.0)
    print("%-22s -> %-10s cmd_v=[%.2f..%.2f] reverse_frames=%d/%d pose=(%.2f,%.2f)"
          % (name, result, vmin, vmax, rev, len(cmds), pose[1], pose[2]))
    return result, rev


def main():
    rospy.init_node("free_rpp_check", anonymous=True)
    rospy.Subscriber("/goal_status", String, status_cb)
    rospy.Subscriber("/cmd_vel", Twist, cmd_cb)
    rospy.Subscriber("/lio_odom", Odometry, odom_cb)
    rospy.wait_for_message("/lio_odom", Odometry, timeout=120)
    rospy.sleep(2.0)
    fails = []

    r1, rev1 = phase("1) forward to A", 1.5, 0.0)
    if "SUCCEEDED" not in r1:
        fails.append("forward leg failed: " + r1)
    if rev1 > 0:
        fails.append("unexpected reverse on a forward leg")

    r2, rev2 = phase("2) behind to B", 0.3, 0.0)
    if "SUCCEEDED" not in r2:
        fails.append("backward leg failed: " + r2)
    if rev2 == 0:
        fails.append("no reverse segment (free+rpp should allow reversing)")

    r3, _ = phase("3) forward to A again", 1.5, 0.0)
    if "SUCCEEDED" not in r3:
        fails.append("return leg failed: " + r3)

    print("=== RESULT ===")
    if fails:
        print("FAIL: " + "; ".join(fails))
        sys.exit(1)
    print("PASS: free+rpp multi-direction (reverse executed)")


if __name__ == "__main__":
    main()
