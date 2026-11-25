#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 反光板绝对定位节点 (T1): 3D 点云强度聚类 → 反光板质心 → 与已标定的反光板地图
# 2D 刚体配准 → 发布 /absolute_pose (map 系绝对位姿+协方差)。
# SLAM 的 absolute.enabled=true 时按门控互补融合 (L19), 与匹配位姿互补。
#
# 原理: 反光板反射强度远高于普通表面 (万集 722 的 intensity 通常 >100),
#       聚类成小簇 → 质心即反光板中心。反光板固定不动, 位置在 map 系已标定。
# 配置: 反光板地图坐标 (map 系) 必须现场标定后写入 YAML。
import math
import itertools
import yaml
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PoseWithCovarianceStamped


class ReflectorAbsolute(object):
    def __init__(self):
        p = rospy.get_param('~')
        self.cloud_topic = p.get('cloud_topic', '/vanjee_points722')
        self.frame = p.get('frame', 'map')
        self.intensity_min = float(p.get('intensity_min', 100.0))   # 强度阈值(现场标定)
        self.link_dist = float(p.get('cluster_link_dist', 0.20))    # 聚类连接距离(m)
        self.min_pts = int(p.get('cluster_min_pts', 3))             # 簇最少点数
        self.max_pts = int(p.get('cluster_max_pts', 300))
        self.min_reflectors = int(p.get('min_reflectors', 2))       # 至少匹配到 2 块才发
        self.residual_max = float(p.get('residual_max', 0.20))      # 配准残差门限(m)
        self.cov_xy = float(p.get('cov_xy', 0.05))                  # 位姿协方差 xy (m^2)
        self.cov_yaw = float(p.get('cov_yaw', 0.03))                # 航向协方差 (rad^2)
        # 反光板地图坐标 (map 系), 现场标定后写入 YAML: [{x:.., y:..}, ...]
        self.known = []
        for r in p.get('reflectors', []):
            self.known.append((float(r['x']), float(r['y'])))
        self.pub = rospy.Publisher(p.get('absolute_topic', '/absolute_pose'),
                                   PoseWithCovarianceStamped, queue_size=2)
        rospy.Subscriber(self.cloud_topic, PointCloud2, self.cb, queue_size=2)
        rospy.loginfo('[reflector] known=%d intensity>=%.0f link=%.2f topic=%s',
                      len(self.known), self.intensity_min, self.link_dist, self.cloud_topic)

    # ---- 反光板检测: 强度阈值 + 邻接聚类 ----
    def detect(self, msg):
        pts = []
        try:
            gen = pc2.read_points(msg, field_names=('x', 'y', 'z', 'intensity'),
                                  skip_nans=True)
            for p in gen:
                if p[3] >= self.intensity_min:
                    pts.append((p[0], p[1]))
        except Exception:
            return []
        if not pts:
            return []
        # 2D 邻接聚类 (union-find)
        n = len(pts)
        parent = list(range(n))

        def find(a):
            while parent[a] != a:
                parent[a] = parent[parent[a]]
                a = parent[a]
            return a

        r2 = self.link_dist * self.link_dist
        # 网格加速: 按 0.2m 网格分桶, 只比邻近桶
        grid = {}
        for i, q in enumerate(pts):
            grid.setdefault((int(math.floor(q[0] / 0.2)),
                             int(math.floor(q[1] / 0.2))), []).append(i)
        for bucket in grid.values():
            for a in range(len(bucket)):
                for b in range(a + 1, len(bucket)):
                    i, j = bucket[a], bucket[b]
                    dx = pts[i][0] - pts[j][0]
                    dy = pts[i][1] - pts[j][1]
                    if dx * dx + dy * dy <= r2:
                        ri, rj = find(i), find(j)
                        if ri != rj:
                            parent[rj] = ri
        groups = {}
        for i in range(n):
            groups.setdefault(find(i), []).append(pts[i])
        out = []
        for g in groups.values():
            if self.min_pts <= len(g) <= self.max_pts:
                xs = [q[0] for q in g]
                ys = [q[1] for q in g]
                out.append((sum(xs) / len(xs), sum(ys) / len(ys)))
        return out

    # ---- 2D 刚体配准: known -> detected, 返回 (x, y, yaw, 残差) 或 None ----
    @staticmethod
    def solve(k, d):
        # k/d: [(x, y)] 等长对应; 最小二乘闭式解 (Umeyama 2D, 无缩放)
        nk, nd = len(k), len(d)
        kx = sum(a[0] for a in k) / nk
        ky = sum(a[1] for a in k) / nk
        dx = sum(a[0] for a in d) / nd
        dy = sum(a[1] for a in d) / nd
        s1 = s2 = 0.0
        for (ka, kb), (da, db) in zip(k, d):
            s1 += ka * da + kb * db
            s2 += ka * db - kb * da
        yaw = math.atan2(s2, s1)
        c, s = math.cos(yaw), math.sin(yaw)
        rx = dx - (c * kx - s * ky)
        ry = dy - (s * kx + c * ky)
        res = 0.0
        for (ka, kb), (da, db) in zip(k, d):
            ex = c * ka - s * kb + rx - da
            ey = s * ka + c * kb + ry - db
            res += ex * ex + ey * ey
        return rx, ry, yaw, math.sqrt(res / max(1, nd))

    def cb(self, msg):
        det = self.detect(msg)
        if len(det) < self.min_reflectors or not self.known:
            return
        # 小集合穷举关联: 检测到的每块板 对应 已知板的不同组合, 残差最小者胜
        best = None
        nk = len(self.known)
        for r in range(self.min_reflectors, min(len(det), nk) + 1):
            for det_combo in itertools.combinations(range(len(det)), r):
                for known_combo in itertools.combinations(range(nk), r):
                    for perm in itertools.permutations(det_combo):
                        pairs_k = [self.known[i] for i in known_combo]
                        pairs_d = [det[i] for i in perm]
                        x, y, yaw, res = self.solve(pairs_k, pairs_d)
                        if best is None or res < best[0]:
                            best = (res, x, y, yaw, r)
        if best is None or best[0] > self.residual_max:
            return
        _, x, y, yaw = best
        m = PoseWithCovarianceStamped()
        m.header.stamp = msg.header.stamp
        m.header.frame_id = self.frame
        m.pose.pose.position.x = x
        m.pose.pose.position.y = y
        m.pose.pose.position.z = 0.0
        m.pose.pose.orientation.z = math.sin(yaw / 2.0)
        m.pose.pose.orientation.w = math.cos(yaw / 2.0)
        cov = 36 * [0.0]
        cov[0] = cov[7] = self.cov_xy
        cov[35] = self.cov_yaw
        m.pose.covariance = cov
        self.pub.publish(m)


def main():
    rospy.init_node('reflector_absolute')
    ReflectorAbsolute()
    rospy.spin()


if __name__ == '__main__':
    main()
