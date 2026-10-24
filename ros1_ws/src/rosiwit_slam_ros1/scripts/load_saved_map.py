#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""load_saved_map.py - 启动时载入已保存地图并切到定位模式。

用法（一般由 slam_simulation.launch 传参）:
  rosrun rosiwit_slam_ros1 load_saved_map.py _map_yaml:=/path/to/map.yaml

参数:
  ~map_yaml      地图 yaml 路径（必需；不存在则只告警，保持建图模式）
  ~initial_x     map 系初始位姿 x（默认 0）
  ~initial_y     map 系初始位姿 y（默认 0）
  ~initial_yaw   map 系初始航向（rad，默认 0）
  ~timeout       等待 /switch_map 的秒数（默认 60）
  ~keep_mapping  失败时不回退到建图（默认 false）

行为:
  1. 读 map.yaml 的 resolution / origin / negate / thresholds；
  2. 调 /switch_map（兼容接口）→ 发布 PGM、载入 .xzmap（或 pcd）并切 localization；
  3. 再补发一次 /initialpose，确保重定位从给定位姿开始；
  4. 失败则回退 /set_slam_mode -> mapping，避免卡在没有地图的定位模式。
"""

import math
import os

import rospy
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped
from xzrobot_ros_dependency.srv import SwitchMap, SwitchMapRequest


def _load_meta(path):
    try:
        with open(path, 'r') as f:
            return yaml.safe_load(f) or {}
    except Exception as e:
        rospy.logerr('[load_saved_map] cannot read %s: %s', path, e)
        return {}


def main():
    rospy.init_node('load_saved_map')

    map_yaml = rospy.get_param('~map_yaml', '')
    initial_x = float(rospy.get_param('~initial_x', 0.0))
    initial_y = float(rospy.get_param('~initial_y', 0.0))
    initial_yaw = float(rospy.get_param('~initial_yaw', 0.0))
    timeout = float(rospy.get_param('~timeout', 60.0))
    keep_mapping = bool(rospy.get_param('~keep_mapping', False))

    if not map_yaml or not os.path.isfile(map_yaml):
        rospy.logwarn('[load_saved_map] map yaml not found: "%s" -> keep mapping mode',
                      map_yaml)
        return

    meta = _load_meta(map_yaml)
    resolution = float(meta.get('resolution', 0.05))
    origin = list(meta.get('origin', [0.0, 0.0, 0.0]))
    while len(origin) < 3:
        origin.append(0.0)

    rospy.loginfo('[load_saved_map] waiting for /switch_map ...')
    try:
        rospy.wait_for_service('/switch_map', timeout=timeout)
    except rospy.ROSException:
        rospy.logerr('[load_saved_map] /switch_map not available within %.0fs', timeout)
        return

    srv = rospy.ServiceProxy('/switch_map', SwitchMap)
    req = SwitchMapRequest()
    req.map_name = os.path.basename(os.path.dirname(map_yaml)) or 'map'
    req.map_dir = map_yaml
    req.map_resolution = resolution
    req.map_origin = [float(origin[0]), float(origin[1]), float(origin[2])]
    req.map_negate = bool(int(meta.get('negate', 0)))
    req.map_occupied_thresh = float(meta.get('occupied_thresh', 0.65))
    req.map_free_thresh = float(meta.get('free_thresh', 0.25))
    req.vir_obs = ''

    pose = PoseWithCovarianceStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = rospy.Time.now()
    pose.pose.pose.position.x = initial_x
    pose.pose.pose.position.y = initial_y
    pose.pose.pose.orientation.z = math.sin(initial_yaw * 0.5)
    pose.pose.pose.orientation.w = math.cos(initial_yaw * 0.5)
    pose.pose.covariance[0] = 0.25
    pose.pose.covariance[7] = 0.25
    pose.pose.covariance[35] = 0.0685
    req.initial_pose = pose

    rospy.loginfo('[load_saved_map] /switch_map %s (res=%.3f origin=[%.2f %.2f %.2f])',
                  map_yaml, resolution, origin[0], origin[1], origin[2])
    resp = srv(req)
    if not resp.success:
        rospy.logerr('[load_saved_map] /switch_map failed; fall back to mapping')
        if not keep_mapping:
            try:
                from rosiwit_slam_ros1.srv import SetSlamMode
                rospy.wait_for_service('/set_slam_mode', timeout=5.0)
                rospy.ServiceProxy('/set_slam_mode', SetSlamMode)('mapping')
            except Exception as e:
                rospy.logwarn('[load_saved_map] fallback set_slam_mode failed: %s', e)
        return

    # 服务已带 initial_pose；再补发一次，覆盖定位线程启动慢的情况。
    pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1, latch=True)
    rospy.sleep(1.0)
    pose.header.stamp = rospy.Time.now()
    pub.publish(pose)
    rospy.loginfo('[load_saved_map] localization mode ready (initial %.2f %.2f %.2f)',
                  initial_x, initial_y, initial_yaw)


if __name__ == '__main__':
    main()
