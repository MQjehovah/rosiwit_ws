#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Isolated end-to-end check for the robot_movebase compat layer.

Runs against a PRIVATE roscore (ROS_MASTER_URI=127.0.0.1:11312) so the real
robot (master on 11311) is never touched: we publish a fake /map, fake /odom,
fake map->base_footprint TF and only observe /navtest/movebase_cmd_vel.
"""
from __future__ import print_function

import math
import sys
import threading
import time

import actionlib
import rospy
import tf2_ros

from geometry_msgs.msg import Pose2D, PoseStamped, TransformStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from std_srvs.srv import Trigger
from xzrobot_ros_dependency.msg import NaviCmdActionAction, NaviCmdActionGoal

ACTION = "/navtest/RobotNaviCmdAction"
CMD_TOPIC = "/navtest/movebase_cmd_vel"

failures = []
feedback_seen = {"percent": 0.0, "navi_state": 0, "count": 0}
last_cmd = {"linear": 0.0, "angular": 0.0}


def check(name, ok, detail=""):
    print(("[PASS] " if ok else "[FAIL] ") + name + (" " + detail if detail else ""))
    if not ok:
        failures.append(name)


def on_feedback(msg):
    feedback_seen["percent"] = msg.execution_percent
    feedback_seen["navi_state"] = msg.navi_state
    feedback_seen["count"] += 1


def on_cmd(msg):
    last_cmd["linear"] = msg.linear.x
    last_cmd["angular"] = msg.angular.z


def publish_static_tf():
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    transform = TransformStamped()
    transform.header.stamp = rospy.Time.now()
    transform.header.frame_id = "map"
    transform.child_frame_id = "base_footprint"
    transform.transform.rotation.w = 1.0
    broadcaster.sendTransform(transform)


def start_odom():
    publisher = rospy.Publisher("/odom", Odometry, queue_size=1)

    def loop():
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            msg = Odometry()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "map"
            msg.child_frame_id = "base_footprint"
            msg.pose.pose.orientation.w = 1.0
            # DWA/MPPI 会按 odom 的当前速度做加速度限制; 假 odom 必须回填
            # 上一条控制量, 否则永远从 0 加速, 只能看到 0.03m/s.
            msg.twist.twist.linear.x = last_cmd["linear"]
            msg.twist.twist.angular.z = last_cmd["angular"]
            try:
                publisher.publish(msg)
            except rospy.ROSException:
                break  # 退出时 publisher 已关闭, 不是测试失败
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                break

    thread = threading.Thread(target=loop)
    thread.setDaemon(True)
    thread.start()


def publish_map():
    publisher = rospy.Publisher("/map", OccupancyGrid, queue_size=1, latch=True)
    grid = OccupancyGrid()
    grid.header.stamp = rospy.Time.now()
    grid.header.frame_id = "map"
    grid.info.resolution = 0.05
    grid.info.width = 200
    grid.info.height = 200
    grid.info.origin.position.x = -5.0
    grid.info.origin.position.y = -5.0
    grid.info.origin.orientation.w = 1.0
    grid.data = [0] * (200 * 200)
    deadline = time.time() + 10.0
    while publisher.get_num_connections() == 0 and time.time() < deadline:
        rospy.sleep(0.1)
    publisher.publish(grid)
    return publisher


def make_goal(cmd, points=None, backing=0.0):
    goal = NaviCmdActionGoal()
    goal.navi_cmd = cmd
    goal.backing_dist = backing
    for (x, y, theta) in (points or []):
        pose = Pose2D()
        pose.x, pose.y, pose.theta = x, y, theta
        goal.tracking_path.append(pose)
    return goal


def wait_result(client, timeout=8.0):
    if not client.wait_for_result(rospy.Duration(timeout)):
        return None
    return client.get_result()


def wait_until(predicate, timeout=4.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if predicate():
            return True
        rospy.sleep(0.05)
    return False


def main():
    rospy.init_node("navtest_driver")
    publish_static_tf()
    publish_map()
    start_odom()
    rospy.Subscriber(CMD_TOPIC, Twist, on_cmd)
    rospy.sleep(0.5)

    client = actionlib.SimpleActionClient(ACTION, NaviCmdActionAction)
    if not client.wait_for_server(rospy.Duration(20.0)):
        print("[FAIL] action server 未出现: " + ACTION)
        return 2
    check("action server 可用", True, ACTION)

    # 1) 不支持的命令必须立即失败
    feedback_seen["count"] = 0
    client.send_goal(make_goal(4), feedback_cb=on_feedback)
    result = wait_result(client)
    check("navi_cmd=4 -> -4",
          result is not None and result.tracking_path_result == -4,
          "result=%s" % (None if result is None else result.tracking_path_result))
    check("navi_cmd=4 无运动指令",
          abs(last_cmd["linear"]) < 1e-6 and abs(last_cmd["angular"]) < 1e-6,
          "v=%.3f w=%.3f" % (last_cmd["linear"], last_cmd["angular"]))

    # 2) 单点命令缺 tracking_path -> -1
    client.send_goal(make_goal(1), feedback_cb=on_feedback)
    result = wait_result(client)
    check("navi_cmd=1 无点位 -> -1",
          result is not None and result.tracking_path_result == -1,
          "result=%s" % (None if result is None else result.tracking_path_result))

    # 3) 原地旋转: 应出现正角速度; 然后取消
    last_cmd["angular"] = 0.0
    feedback_seen["count"] = 0
    client.send_goal(make_goal(6), feedback_cb=on_feedback)
    rotated = wait_until(lambda: last_cmd["angular"] > 0.05)
    feedback_ok = wait_until(lambda: feedback_seen["count"] > 0)
    check("navi_cmd=6 输出正角速度", rotated, "w=%.3f" % last_cmd["angular"])
    check("navi_cmd=6 有 feedback", feedback_ok, "count=%d" % feedback_seen["count"])
    client.cancel_goal()
    finished = client.wait_for_result(rospy.Duration(8.0))
    check("旋转取消 -> PREEMPTED",
          finished and client.get_state() == actionlib.GoalStatus.PREEMPTED,
          "finished=%s state=%s" % (finished, client.get_state()))

    # 4) 倒车: 应出现负线速度; 然后取消
    rospy.sleep(0.5)
    last_cmd["linear"] = 0.0
    client.send_goal(make_goal(7, backing=0.3), feedback_cb=on_feedback)
    backed = wait_until(lambda: last_cmd["linear"] < -0.01)
    check("navi_cmd=7 输出负线速度", backed, "v=%.3f" % last_cmd["linear"])
    client.cancel_goal()
    finished = client.wait_for_result(rospy.Duration(8.0))
    check("倒车取消 -> PREEMPTED",
          finished and client.get_state() == actionlib.GoalStatus.PREEMPTED,
          "finished=%s state=%s" % (finished, client.get_state()))

    # 5) 两点导航: 在有地图/odom/TF 的前提下应进入规划->控制, 出现正线速度
    rospy.sleep(0.5)
    last_cmd["linear"] = 0.0
    feedback_seen["percent"] = 0.0
    client.send_goal(make_goal(2, [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)]),
                     feedback_cb=on_feedback)
    moved = wait_until(lambda: last_cmd["linear"] > 0.05, timeout=6.0)
    check("navi_cmd=2 规划后输出正线速度", moved, "v=%.3f" % last_cmd["linear"])
    client.cancel_goal()
    finished = client.wait_for_result(rospy.Duration(8.0))
    check("两点导航取消 -> PREEMPTED",
          finished and client.get_state() == actionlib.GoalStatus.PREEMPTED,
          "finished=%s state=%s" % (finished, client.get_state()))

    # 6) 取消后不得自己恢复导航 (回归: 在途规划结果被丢弃)
    rospy.sleep(1.0)
    last_cmd["linear"] = 0.0
    rospy.sleep(1.0)
    check("取消后保持停止",
          abs(last_cmd["linear"]) < 1e-6 and abs(last_cmd["angular"]) < 1e-6,
          "v=%.3f w=%.3f" % (last_cmd["linear"], last_cmd["angular"]))

    # 7) 稠密路径跟踪
    rospy.sleep(0.5)
    last_cmd["linear"] = 0.0
    client.send_goal(make_goal(3, [(0.0, 0.0, 0.0), (0.5, 0.0, 0.0), (1.0, 0.0, 0.0)]),
                     feedback_cb=on_feedback)
    followed = wait_until(lambda: last_cmd["linear"] > 0.05, timeout=6.0)
    check("navi_cmd=3 跟踪路径输出正线速度", followed, "v=%.3f" % last_cmd["linear"])
    client.cancel_goal()
    finished = client.wait_for_result(rospy.Duration(8.0))
    check("路径跟踪取消 -> PREEMPTED",
          finished and client.get_state() == actionlib.GoalStatus.PREEMPTED,
          "finished=%s state=%s" % (finished, client.get_state()))


    # 8) RViz 2D Nav Goal: /move_base_simple/goal (原生目标, 走 ActionTask)
    rospy.sleep(0.5)
    last_cmd["linear"] = 0.0
    goal_pub = rospy.Publisher("/navtest/move_base_simple/goal", PoseStamped,
                               queue_size=1)
    rospy.sleep(0.3)
    rviz_goal = PoseStamped()
    rviz_goal.header.frame_id = "map"
    rviz_goal.header.stamp = rospy.Time.now()
    rviz_goal.pose.position.x = 1.0
    rviz_goal.pose.orientation.w = 1.0
    goal_pub.publish(rviz_goal)
    moved = wait_until(lambda: last_cmd["linear"] > 0.05, timeout=6.0)
    check("RViz goal 触发导航", moved, "v=%.3f" % last_cmd["linear"])
    try:
        stop_nav = rospy.ServiceProxy("/nav_stop", Trigger)
        stop_nav()
        rospy.sleep(0.8)
        check("RViz goal 可被 /nav_stop 停止",
              abs(last_cmd["linear"]) < 1e-6 and abs(last_cmd["angular"]) < 1e-6,
              "v=%.3f w=%.3f" % (last_cmd["linear"], last_cmd["angular"]))
    except rospy.ServiceException as exc:
        check("RViz goal 可被 /nav_stop 停止", False, str(exc))

    print("")
    if failures:
        print("FAILED: %d 项 -> %s" % (len(failures), ", ".join(failures)))
        return 1
    print("ALL PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
