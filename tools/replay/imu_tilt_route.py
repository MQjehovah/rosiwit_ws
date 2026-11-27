#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""用 IMU 重力方向独立核对"地图倾斜量": 沿回放路线, 把每个 IMU 样本的重力方向按节点当时的航向 (来自节点日志的
[locstate]) 转到 map 系, 分窗平均 -> map 竖直 vs 真实重力的夹角; 与同一时段匹配器估计的 map_tilt 并排。
    python2 imu_tilt_route.py <bag> <node_log> [window_s=20]
车在动, 加速度计 = 重力 + 运动加速度; 运动加速度在窗内近似零均值, 窗平均后只剩重力 (+零偏 + URDF 安装角误差)。
"""
from __future__ import print_function
import math
import re
import io
import sys

import rosbag

bag_path, log_path = sys.argv[1], sys.argv[2]
win = float(sys.argv[3]) if len(sys.argv) > 3 else 20.0

# URDF: base_link<-lidar rpy = (0.687, 0.020, 1.569) rad  -> 车体 +z (竖直) 在雷达系里的方向
phi, th, ps = 0.687, 0.020, 1.569
def mm(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def Rx(a):
    c, s_ = math.cos(a), math.sin(a)
    return [[1, 0, 0], [0, c, -s_], [0, s_, c]]


def Ry(a):
    c, s_ = math.cos(a), math.sin(a)
    return [[c, 0, s_], [0, 1, 0], [-s_, 0, c]]


def Rz(a):
    c, s_ = math.cos(a), math.sin(a)
    return [[c, -s_, 0], [s_, c, 0], [0, 0, 1]]


Rt = mm(mm(Rx(-phi), Ry(-th)), Rz(-ps))      # R_bl^T: 雷达系 -> 车体系
zbody = [Rt[i][2] for i in range(3)]          # 车体竖直 = R_bl^T e_z (由矩阵算, 不手推)

ansi = re.compile(r'\x1b\[[0-9;]*m')
yaw_t, yaw_v, healthy = [], [], []
tilt_t, tilt_v = [], []
# [quality] 行没有时间戳: 按它在日志里的位置, 前后两条 [locstate] (带 ROS 时间 = 录包时间) 之间线性插值
pending, prev_t = [], None
for line in io.open(log_path, encoding="utf-8", errors="replace"):
    s = ansi.sub('', line)
    q = re.match(r'\[quality\].*?map_tilt=([\d.]+)', s)
    if q:
        pending.append(float(q.group(1)))
        continue
    m = re.search(r'\[ INFO\] \[[\d.]+, ([\d.]+)\]: \[locstate\] mode=\S+ st=(\d) matching=[\d.]+ pos=\((-?[\d.]+),(-?[\d.]+),(-?[\d.]+)deg\)', s)
    if m:
        t = float(m.group(1))
        if prev_t is not None and pending:
            for k, v in enumerate(pending):
                tilt_t.append(prev_t + (t - prev_t) * (k + 0.5) / len(pending))
                tilt_v.append(v)
        pending, prev_t = [], t
        yaw_t.append(t)
        yaw_v.append(math.radians(float(m.group(4))))
        healthy.append(m.group(2) == '1')
if not yaw_t:
    print('日志里没有 [locstate]'); sys.exit(1)
# 航向解卷绕
for i in range(1, len(yaw_v)):
    while yaw_v[i] - yaw_v[i - 1] > math.pi:
        yaw_v[i] -= 2 * math.pi
    while yaw_v[i] - yaw_v[i - 1] < -math.pi:
        yaw_v[i] += 2 * math.pi


def yaw_at(t):
    import bisect
    i = bisect.bisect_left(yaw_t, t)
    if i <= 0 or i >= len(yaw_t):
        return None
    a = (t - yaw_t[i - 1]) / max(1e-6, yaw_t[i] - yaw_t[i - 1])
    return yaw_v[i - 1] + a * (yaw_v[i] - yaw_v[i - 1])


def healthy_at(t):
    import bisect
    i = bisect.bisect_right(yaw_t, t) - 1
    return i >= 0 and healthy[i]


samples = []      # (t, gx, gy, gz) in map frame
n_imu = 0
for topic, msg, _ in rosbag.Bag(bag_path).read_messages(topics=['/vanjee_lidar_imu_packets']):
    n_imu += 1
    t = msg.header.stamp.to_sec()
    y = yaw_at(t)
    if y is None or not healthy_at(t):
        continue
    a = msg.linear_acceleration
    n = math.sqrt(a.x * a.x + a.y * a.y + a.z * a.z)
    v = (a.x / n, a.y / n, a.z / n)
    # 雷达系 -> 车体系 用 R_bl (= Rt 的转置): v_body = R_bl v_lidar. Rt 本身是反过来 (车体系 -> 雷达系)
    bx = Rt[0][0] * v[0] + Rt[1][0] * v[1] + Rt[2][0] * v[2]
    by = Rt[0][1] * v[0] + Rt[1][1] * v[1] + Rt[2][1] * v[2]
    bz = Rt[0][2] * v[0] + Rt[1][2] * v[1] + Rt[2][2] * v[2]
    c, s = math.cos(y), math.sin(y)
    samples.append((t, c * bx - s * by, s * bx + c * by, bz))
print('IMU 样本 %d 个, 其中定位健康且有航向的 %d 个 (%.0f s); [locstate] %d 条' % (
    n_imu, len(samples), (samples[-1][0] - samples[0][0]) if samples else 0, len(yaw_t)))


def summarize(sel, name):
    if not sel:
        print('%s: 无样本' % name); return
    gx = sum(p[1] for p in sel) / len(sel)
    gy = sum(p[2] for p in sel) / len(sel)
    gz = sum(p[3] for p in sel) / len(sel)
    ang = math.degrees(math.acos(max(-1.0, min(1.0, gz / math.sqrt(gx * gx + gy * gy + gz * gz)))))
    an = math.hypot(gx, gy)
    vx, vy = (ang * (-gy) / an, ang * gx / an) if an > 1e-9 else (0.0, 0.0)    # z x g
    print('%s: IMU 给出 map 竖直 vs 重力 %.2f°, 旋转向量 (%.2f°, %.2f°)' % (name, ang, vx, vy))


def matcher(t0, t1):
    sel = [v for t, v in zip(tilt_t, tilt_v) if t0 <= t <= t1]
    return (sum(sel) / len(sel), len(sel)) if sel else (float('nan'), 0)


if samples:
    t0, t1 = samples[0][0], samples[-1][0]
    summarize(samples, '全程')
    m, k = matcher(t0, t1)
    print('        匹配器 map_tilt 全程均值 %.2f° (n=%d)' % (m, k))
    print('--- 每 %.0f s 一窗 ---' % win)
    t = t0
    while t < t1:
        sel = [p for p in samples if t <= p[0] < t + win]
        m, k = matcher(t, t + win)
        if sel:
            gx = sum(p[1] for p in sel) / len(sel)
            gy = sum(p[2] for p in sel) / len(sel)
            gz = sum(p[3] for p in sel) / len(sel)
            ang = math.degrees(math.acos(max(-1.0, min(1.0, gz / math.sqrt(gx * gx + gy * gy + gz * gz)))))
            an = math.hypot(gx, gy)
            vx, vy = (ang * (-gy) / an, ang * gx / an) if an > 1e-9 else (0.0, 0.0)
            yaws = [math.degrees(yaw_at(p[0])) for p in sel[::max(1, len(sel) // 10)]]
            print('  t=%4.0fs  IMU: %.2f° (%.2f°, %.2f°)   匹配器: %.2f° (n=%d)   航向 %+.0f~%+.0f°' % (
                t - t0, ang, vx, vy, m, k, min(yaws), max(yaws)))
        t += win
