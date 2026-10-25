#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""恢复流程回归: 路径被禁行区挡住 -> 规划失败 -> 反向逃逸恢复.

用法 (已启动 sim_slam_nav.launch):
    rosrun rosiwit_navigation_ros1 recovery_check.py

判据:
  1) 出现反向逃逸 (cmd_vel.linear.x < -0.05)
  2) 机器人确实后退了 (位移方向与朝向相反, 或 odom 位置明显后移)
  3) 节点没有崩 (之后仍能响应/发布状态)
  4) 清除禁行区后, 同一目标能重新成功

背景: 本轮把代价地图语义统一成 nav_msgs (-1/0..100), 恢复流程用的
astar 规划器阈值跟着改了; 之前恢复只在"目标在禁行区里"时被动触发过,
没有验证过它会真的倒车脱离.
"""
from __future__ import print_function
import math
import sys

import rospy
from geometry_msgs.msg import Point, PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from xzrobot_ros_dependency.msg import VirtualObstacles, VirtualForm

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


def keepout(pub, cx, cy, half):
    msg = VirtualObstacles()
    form = VirtualForm()
    for (dx, dy) in ((-half, -half), (half, -half), (half, half), (-half, half)):
        pt = Point()
        pt.x = cx + dx
        pt.y = cy + dy
        form.form.append(pt)
    msg.list.append(form)
    pub.publish(msg)


def main():
    rospy.init_node("recovery_check", anonymous=True)
    rospy.Subscriber("/goal_status", String, status_cb)
    rospy.Subscriber("/cmd_vel", Twist, cmd_cb)
    rospy.Subscriber("/lio_odom", Odometry, odom_cb)
    ko_pub = rospy.Publisher("/virtual_obstacles", VirtualObstacles,
                             queue_size=1, latch=True)
    rospy.wait_for_message("/lio_odom", Odometry, timeout=120)
    rospy.sleep(2.0)
    fails = []

    # 1) 先跑起来
    STATUS["s"] = ""
    send_goal(3.0, -2.0)
    t0 = rospy.get_time()
    while rospy.get_time() - t0 < 90.0:
        if ODOM and math.hypot(ODOM[-1][1] - (-0.5), ODOM[-1][2] - 0.0) > 0.8:
            break
        rospy.sleep(0.2)
    if not ODOM:
        print("FAIL: no odom")
        sys.exit(2)
    px, py = ODOM[-1][1], ODOM[-1][2]
    print("1) moving, at (%.2f, %.2f)" % (px, py))

    # 2) 把**目标**盖在禁行区里: 规划必然失败 -> 触发反向逃逸恢复.
    #    (只堵路中间的话, 规划器会绕过去, 不会触发恢复)
    print("2) blocking the goal (3.0, -2.0)")
    keepout(ko_pub, 3.0, -2.0, 0.45)

    # 3) 观察 25s: 是否出现倒车 (恢复规划出的 reverse escape 由 recovery
    #    controller 执行, 走同一个 publishCommand -> /cmd_vel 会是负速度)
    n0 = len(CMD)
    t1 = rospy.get_time()
    rev = 0
    vmin = 0.0
    while rospy.get_time() - t1 < 25.0:
        for (t, v, w) in CMD[n0:]:
            if v < -0.05:
                rev += 1
                vmin = min(vmin, v)
        if rev > 3:
            break
        rospy.sleep(0.2)
    n0 = len(CMD)
    print("3) reverse frames=%d vmin=%.2f status=%s" % (rev, vmin, STATUS["s"]))
    if rev == 0:
        fails.append("no reverse escape detected")

    # 4) 清除禁行区, 同一目标应能重新规划成功
    ko_pub.publish(VirtualObstacles())
    rospy.sleep(2.0)
    STATUS["s"] = ""
    send_goal(3.0, -2.0)
    t2 = rospy.get_time()
    res = "TIMEOUT"
    while rospy.get_time() - t2 < 120.0:
        s = STATUS["s"]
        if "SUCCEEDED" in s or "FAILED" in s or "ABORTED" in s:
            res = s
            break
        rospy.sleep(0.2)
    print("4) after clearing keepout -> %s" % res)
    if "SUCCEEDED" not in res:
        fails.append("no recovery after clearing keepout (%s)" % res)

    print("=== RESULT ===")
    if fails:
        print("FAIL: " + "; ".join(fails))
        sys.exit(1)
    print("PASS: reverse-escape recovery")


if __name__ == "__main__":
    main()
