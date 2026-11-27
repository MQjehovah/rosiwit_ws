#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""从停车位录包 dock.bag 取: (1) 几帧扫描 (雷达系, 节点 [DumpSeq] 同格式) + 节点当时的"水平"雷达位姿 (猜测);
(2) 静止时 IMU 的重力方向 -> 用节点发布的位姿转到 map 系, 得到"map 竖直 vs 真实重力"的夹角。
    python2 dock_extract.py <dock.bag> <out_dir>
"""
from __future__ import print_function
import math
import os
import struct
import sys

import rosbag
import rospy
import tf2_ros
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import TransformStamped

bag_path, out_dir = sys.argv[1], sys.argv[2]
if not os.path.isdir(out_dir):
    os.makedirs(out_dir)
bag = rosbag.Bag(bag_path)
buf = tf2_ros.Buffer(cache_time=rospy.Duration(3600.0))
imu_frame = [None]
acc = []
scans = []
frames = set()
for topic, msg, t in bag.read_messages(topics=['/tf', '/tf_static', '/vanjee_lidar_imu_packets', '/vanjee_points722']):
    if topic == '/tf' or topic == '/tf_static':
        for tr in msg.transforms:
            frames.add((tr.header.frame_id, tr.child_frame_id))
            ts = TransformStamped()
            ts.header = tr.header
            ts.child_frame_id = tr.child_frame_id
            ts.transform = tr.transform
            if topic == '/tf_static':
                buf.set_transform_static(ts, 'bag')
            else:
                buf.set_transform(ts, 'bag')
    elif topic == '/vanjee_lidar_imu_packets':
        imu_frame[0] = msg.header.frame_id
        a = msg.linear_acceleration
        acc.append((msg.header.stamp.to_sec(), a.x, a.y, a.z))
    else:
        scans.append((topic, msg, t))

# ---- TF 树 ----
print('TF 边: ' + ', '.join(sorted('%s->%s' % e for e in frames)))
print('IMU frame: %s, %d 个样本, %d 帧扫描' % (imu_frame[0], len(acc), len(scans)))

# ---- IMU 重力 ----
n = len(acc)
mx = sum(a[1] for a in acc) / n
my = sum(a[2] for a in acc) / n
mz = sum(a[3] for a in acc) / n
norm = math.sqrt(mx * mx + my * my + mz * mz)
gx, gy, gz = mx / norm, my / norm, mz / norm
tilt_imu = math.degrees(math.acos(max(-1.0, min(1.0, -gz))))
print('IMU 平均加速度 (m/s^2): (%.4f, %.4f, %.4f) |a|=%.4f' % (mx, my, mz, norm))
# URDF 安装: base_link<-lidar rpy=(0.687, 0.020, 1.569) rad -> 车体 +z (竖直) 在雷达系里的方向
phi, th, ps = 0.687, 0.020, 1.569
def _mm(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
def _Rx(a):
    c, s_ = math.cos(a), math.sin(a); return [[1, 0, 0], [0, c, -s_], [0, s_, c]]
def _Ry(a):
    c, s_ = math.cos(a), math.sin(a); return [[c, 0, s_], [0, 1, 0], [-s_, 0, c]]
def _Rz(a):
    c, s_ = math.cos(a), math.sin(a); return [[c, -s_, 0], [s_, c, 0], [0, 0, 1]]
_Rt = _mm(_mm(_Rx(-phi), _Ry(-th)), _Rz(-ps))     # 雷达系 -> 车体系
zbody = tuple(_Rt[i][2] for i in range(3))         # 车体竖直 = R_bl^T e_z (由矩阵算, 不手推)
dot = zbody[0] * gx + zbody[1] * gy + zbody[2] * gz
print('URDF 推出的车体竖直方向(雷达系): (%.4f, %.4f, %.4f)' % zbody)
print('实测重力方向与它的夹角 (车体相对 URDF 姿态的物理倾斜): %.3f°' % math.degrees(math.acos(max(-1.0, min(1.0, dot)))))

# 重力转到 map 系 (用节点发布的位姿): map 竖直 vs 重力 的夹角 = 这块地面上"匹配器应该看到的倾斜"
t0 = scans[len(scans) // 2][2]
for src_frame in [imu_frame[0], 'lidar']:
    try:
        tr = buf.lookup_transform('map', src_frame, t0)
    except Exception as e:
        print('lookup map<-%s 失败: %s' % (src_frame, str(e)[:80]))
        continue
    q = tr.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    # g_map = R * g_imu
    r00, r01, r02 = 1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)
    r10, r11, r12 = 2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)
    r20, r21, r22 = 2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)
    gmx, gmy, gmz = r00 * gx + r01 * gy + r02 * gz, r10 * gx + r11 * gy + r12 * gz, r20 * gx + r21 * gy + r22 * gz
    ang = math.degrees(math.acos(max(-1.0, min(1.0, gmz))))   # g 是竖直向上方向, 与 map +z 的夹角
    # 把 map +z 转到实测竖直方向的旋转 (绕水平轴): 轴 = z x g 归一化 = (-g_y, g_x, 0)
    ax, ay = -gmy, gmx
    an = math.hypot(ax, ay)
    print('map<-%s: 重力(向上)在 map 系 (%.5f, %.5f, %.5f); map 竖直 vs 真实重力 夹角 %.3f°, 对应倾斜旋转向量 (%.3f°, %.3f°, 0) [map 系 x/y 分量, 与 map_tilt 同口径]' % (
        src_frame, gmx, gmy, gmz, ang, (ang * ax / an) if an > 1e-9 else 0.0,
        (ang * ay / an) if an > 1e-9 else 0.0))

# ---- 扫描 ----
def write_pcd(path, cloud):
    pts = []
    for p in pc2.read_points(cloud, field_names=('x', 'y', 'z'), skip_nans=True):
        pts.append(p)
    inten = []
    names = [f.name for f in cloud.fields]
    if 'intensity' in names:
        inten = [p[0] for p in pc2.read_points(cloud, field_names=('intensity',), skip_nans=True)]
    with open(path, 'wb') as f:
        f.write('# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z intensity normal_x normal_y normal_z curvature\n'
                'SIZE 4 4 4 4 4 4 4 4\nTYPE F F F F F F F F\nCOUNT 1 1 1 1 1 1 1 1\nWIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA binary\n'
                % (len(pts), len(pts)))
        for i, p in enumerate(pts):
            f.write(struct.pack('<8f', p[0], p[1], p[2], inten[i] if i < len(inten) else 0.0, 0.0, 0.0, 0.0, 0.0))
    return len(pts)


idx = [10, len(scans) // 2, len(scans) - 1]
lines = []
for k in idx:
    topic, cloud, t = scans[k]
    tr = buf.lookup_transform('map', 'lidar', cloud.header.stamp)
    q = tr.transform.rotation
    v = tr.transform.translation
    d = os.path.join(out_dir, 'f_%02d' % k)
    if not os.path.isdir(d):
        os.makedirs(d)
    npts = write_pcd(os.path.join(d, 'scan.pcd'), cloud)
    with open(os.path.join(d, 'poses.txt'), 'w') as f:
        f.write('guess_trans %.9f %.9f %.9f\nguess_rot %.9f %.9f %.9f %.9f\n' % (v.x, v.y, v.z, q.x, q.y, q.z, q.w))
    print('f_%02d: %d 点, 雷达水平位姿 (%.4f, %.4f, %.4f) 四元数 (%.6f, %.6f, %.6f, %.6f), 扫描 frame=%s' % (
        k, npts, v.x, v.y, v.z, q.x, q.y, q.z, q.w, cloud.header.frame_id))
bag.close()
