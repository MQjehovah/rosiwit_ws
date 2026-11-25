#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 反光板绝对定位 / 自动标定节点 (T1)
#
# 两种模式 (param ~mode):
#   locate    (默认) 用当前位姿把检测到的反光板转到 map 系, 与已知反光板匹配,
#             求修正 C, 发布 C ∘ 当前位姿 到 /absolute_pose。
#   calibrate 定位正确时, 把检测到的反光板转到 map 系并聚类, 自动输出稳定坐标,
#             直接填进 config/reflector.yaml, **不需要人工量 xy**。
#
# 为什么需要位姿: 雷达点云是 lidar 系 (安装 pitch 39.37°), 不能直接取 x,y。
# 变换链: lidar -> base_footprint (静态外参, 见 lidar_in_base) -> map (curr_localization_pose)。
# locate 模式下用"当前位姿"只做坐标搬运, 匹配结果 C 是绝对修正, 不会自证。
import math
import itertools
import os
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped


def rpy_to_R(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ])


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def deg2rad(d):
    return d * math.pi / 180.0


class ReflectorNode(object):
    def __init__(self):
        p = rospy.get_param('~')
        self.mode = str(p.get('mode', 'locate'))
        self.calibrate = (self.mode == 'calibrate')
        self.cloud_topic = p.get('cloud_topic', '/vanjee_points722')
        self.pose_topic = p.get('pose_topic', '/curr_localization_pose')
        self.frame = p.get('frame', 'map')
        self.absolute_topic = p.get('absolute_topic', '/absolute_pose')

        # ---- 检测参数 (现场标定) ----
        self.intensity_min = float(p.get('intensity_min', 100.0))
        self.link_dist = float(p.get('cluster_link_dist', 0.20))
        self.min_pts = int(p.get('cluster_min_pts', 3))
        self.max_pts = int(p.get('cluster_max_pts', 300))
        self.min_reflectors = int(p.get('min_reflectors', 2))
        self.residual_max = float(p.get('residual_max', 0.20))
        self.cov_xy = float(p.get('cov_xy', 0.05))
        self.cov_yaw = float(p.get('cov_yaw', 0.03))
        self.pose_timeout = float(p.get('pose_timeout', 1.0))

        # ---- lidar -> base_footprint 静态外参 (URDF: base_link->lidar + base_footprint z=0.07) ----
        lb = p.get('lidar_in_base', {}) or {}
        self.R_lb = rpy_to_R(deg2rad(float(lb.get('roll', 39.37))),
                             deg2rad(float(lb.get('pitch', 1.13))),
                             deg2rad(float(lb.get('yaw', 89.9))))
        self.t_lb = np.array([float(lb.get('x', 0.4145)),
                              float(lb.get('y', 0.0193)),
                              float(lb.get('z', 0.9844))])

        # ---- 已知反光板 (map 系) ----
        self.known = []
        for r in p.get('reflectors', []):
            self.known.append((float(r['x']), float(r['y'])))

        # ---- 标定模式 ----
        self.calib_dist = float(p.get('calib_cluster_dist', 0.5))
        self.calib_min_obs = int(p.get('calib_min_obs', 20))
        self.calib_max_std = float(p.get('calib_max_std', 0.15))
        self.calib_min_interval = float(p.get('calib_min_interval', 0.3))
        self.calib_report_sec = float(p.get('calib_report_sec', 5.0))
        self.output_yaml = p.get('output_yaml', '~/.ros/reflector_calib.yaml')
        self.clusters = []          # [{sx,sy,sxx,syy,n}]
        self._last_report = 0.0

        self.pose = None            # (x, y, yaw, stamp)
        self.pub = rospy.Publisher(self.absolute_topic, PoseWithCovarianceStamped, queue_size=2)
        rospy.Subscriber(self.pose_topic, PoseStamped, self.pose_cb, queue_size=2)
        rospy.Subscriber(self.cloud_topic, PointCloud2, self.cloud_cb, queue_size=2)
        rospy.on_shutdown(self.save_calib)
        rospy.loginfo('[reflector] mode=%s known=%d intensity>=%.0f cloud=%s pose=%s',
                      self.mode, len(self.known), self.intensity_min,
                      self.cloud_topic, self.pose_topic)

    # ---------- 输入 ----------
    def pose_cb(self, msg):
        q = msg.pose.orientation
        self.pose = (msg.pose.position.x, msg.pose.position.y, yaw_of(q), msg.header.stamp.to_sec())

    def detect(self, msg):
        """强度阈值 + 3D 邻接聚类 -> lidar 系质心列表"""
        pts = []
        try:
            gen = pc2.read_points(msg, field_names=('x', 'y', 'z', 'intensity'),
                                  skip_nans=True)
            for p in gen:
                if p[3] >= self.intensity_min:
                    pts.append((p[0], p[1], p[2]))
        except Exception as e:
            rospy.logwarn_throttle(5.0, '[reflector] read_points: %s', e)
            return []
        if not pts:
            return []
        n = len(pts)
        parent = list(range(n))

        def find(a):
            while parent[a] != a:
                parent[a] = parent[parent[a]]
                a = parent[a]
            return a

        l2 = self.link_dist * self.link_dist
        for i in range(n):
            xi, yi, zi = pts[i]
            for j in range(i + 1, n):
                xj, yj, zj = pts[j]
                dx, dy, dz = xi - xj, yi - yj, zi - zj
                if dx * dx + dy * dy + dz * dz <= l2:
                    ri, rj = find(i), find(j)
                    if ri != rj:
                        parent[ri] = rj
        groups = {}
        for i in range(n):
            groups.setdefault(find(i), []).append(pts[i])
        out = []
        for g in groups.values():
            if len(g) < self.min_pts or len(g) > self.max_pts:
                continue
            k = float(len(g))
            out.append((sum(q[0] for q in g) / k,
                        sum(q[1] for q in g) / k,
                        sum(q[2] for q in g) / k))
        return out

    def to_map(self, p):
        """lidar 系点 -> map 系 (x,y); 忽略 z"""
        b = self.R_lb.dot(np.array(p, dtype=float)) + self.t_lb
        x, y, yaw, _ = self.pose
        c, s = math.cos(yaw), math.sin(yaw)
        return (c * b[0] - s * b[1] + x, s * b[0] + c * b[1] + y)

    # ---------- 配准 (已知 -> 检测) ----------
    @staticmethod
    def solve(k, d):
        """2D 刚体配准: k 对齐到 d, 返回 (rx, ry, yaw, rms)"""
        n = float(len(k))
        kx = sum(a[0] for a in k) / n
        ky = sum(a[1] for a in k) / n
        dx = sum(a[0] for a in d) / n
        dy = sum(a[1] for a in d) / n
        s1 = s2 = 0.0
        for (ka, kb), (da, db) in zip(k, d):
            ka -= kx; kb -= ky
            da -= dx; db -= dy
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
        return rx, ry, yaw, math.sqrt(res / max(1, n))

    def match(self, det_map):
        """穷举关联, 返回把**检测点对齐到已知点**的最优 C (rx, ry, yaw, rms) 或 None

        方向约定: 检测点是用"当前位姿"搬到 map 系的, 带着当前位姿的误差 E;
        C = E^-1, 所以绝对位姿 = C ∘ 当前位姿。"""
        best, nk = None, len(self.known)
        for r in range(self.min_reflectors, min(len(det_map), nk) + 1):
            for kc in itertools.combinations(range(nk), r):
                for dc in itertools.combinations(range(len(det_map)), r):
                    for perm in itertools.permutations(dc):
                        kk = [self.known[i] for i in kc]
                        dd = [det_map[i] for i in perm]
                        x, y, yaw, res = self.solve(dd, kk)
                        if best is None or res < best[3]:
                            best = (x, y, yaw, res)
        if best is None or best[3] > self.residual_max:
            return None
        return best

    # ---------- 主回调 ----------
    def cloud_cb(self, msg):
        if self.pose is None:
            return
        if msg.header.stamp.to_sec() - self.pose[3] > self.pose_timeout:
            rospy.logwarn_throttle(5.0, '[reflector] pose stale, skip')
            return
        det = self.detect(msg)
        if not det:
            return
        det_map = [self.to_map(p) for p in det]
        if self.calibrate:
            self.accumulate(det_map, msg.header.stamp.to_sec())
            return
        if not self.known or len(det_map) < self.min_reflectors:
            return
        m = self.match(det_map)
        if m is None:
            return
        rx, ry, yaw_c, rms = m
        # 绝对位姿 = C ∘ 当前位姿
        c, s = math.cos(yaw_c), math.sin(yaw_c)
        px, py, pyaw, _ = self.pose
        out_x = c * px - s * py + rx
        out_y = s * px + c * py + ry
        out_yaw = pyaw + yaw_c
        self.publish(out_x, out_y, out_yaw, msg.header.stamp, rms)

    def publish(self, x, y, yaw, stamp, rms):
        m = PoseWithCovarianceStamped()
        m.header.stamp = stamp
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
        rospy.loginfo_throttle(2.0, '[reflector] absolute x=%.3f y=%.3f yaw=%.1fdeg rms=%.3fm',
                               x, y, math.degrees(yaw), rms)

    # ---------- 标定 ----------
    def accumulate(self, pts, now):
        """标定累积: 每个簇最多每 calib_min_interval 秒记一次 (避免 1 秒数据就"合格")"""
        for (x, y) in pts:
            best, bd = None, self.calib_dist * self.calib_dist
            for c in self.clusters:
                d2 = (x - c['sx'] / c['n']) ** 2 + (y - c['sy'] / c['n']) ** 2
                if d2 < bd:
                    best, bd = c, d2
            if best is None:
                self.clusters.append({'sx': x, 'sy': y, 'sxx': x * x, 'syy': y * y,
                                      'n': 1, 't': now})
                continue
            if now - best['t'] < self.calib_min_interval:
                continue          # 同一簇短时间内重复观测不计数
            best['sx'] += x
            best['sy'] += y
            best['sxx'] += x * x
            best['syy'] += y * y
            best['n'] += 1
            best['t'] = now
        if now - self._last_report >= self.calib_report_sec:
            self._last_report = now
            for i, c in enumerate(self.stable(), 1):
                rospy.loginfo('[reflector_calib] R%d x=%.3f y=%.3f obs=%d',
                              i, c[0], c[1], c[2])

    def stable(self):
        """稳定簇: (x, y, n), 按观测次数降序"""
        out = []
        for c in self.clusters:
            n = c['n']
            if n < self.calib_min_obs:
                continue
            nf = float(n)
            mx, my = c['sx'] / nf, c['sy'] / nf
            vx = max(0.0, c['sxx'] / nf - mx * mx)
            vy = max(0.0, c['syy'] / nf - my * my)
            if math.sqrt(vx + vy) > self.calib_max_std:
                continue
            out.append((mx, my, n))
        out.sort(key=lambda t: -t[2])
        return out

    def save_calib(self):
        if not self.calibrate:
            return
        pts = self.stable()
        path = os.path.expanduser(self.output_yaml)
        d = os.path.dirname(path)
        if d and not os.path.isdir(d):
            try:
                os.makedirs(d)
            except Exception:
                pass
        try:
            with open(path, 'w') as f:
                f.write('# 自动标定结果 (reflector mode=calibrate)\n')
                f.write('# 用法: 复制 reflectors 到 config/reflector.yaml, 然后 mode=locate 启动\n')
                if not pts:
                    f.write('# 没有稳定簇 (obs>=%d, std<%.2f)\n' % (self.calib_min_obs, self.calib_max_std))
                f.write('reflectors:\n')
                for (x, y, n) in pts:
                    f.write('  - {x: %.3f, y: %.3f}   # obs=%d\n' % (x, y, n))
            rospy.loginfo('[reflector_calib] saved %d reflectors -> %s', len(pts), path)
        except Exception as e:
            rospy.logerr('[reflector_calib] save failed: %s', e)


def main():
    rospy.init_node('reflector_absolute')
    ReflectorNode()
    rospy.spin()


if __name__ == '__main__':
    main()
