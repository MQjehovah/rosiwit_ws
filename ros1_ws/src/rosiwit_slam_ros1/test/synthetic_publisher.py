#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""合成数据发布器（计划 T13）

不需要机器人、不需要真实雷达，就能验证旧接口的行为面：
假 IMU（默认 100Hz）+ 假 PointCloud2（默认 10Hz），场景是一个四周封闭的"房间"。

为了能让 FAST-LIO2 真正初始化，IMU 不是完全静止的：机器人绕 Z 轴以
`_yaw_rate_deg` 匀速自转，点云按 -yaw(t) 反向旋转（等价于"机器人在转、房间不动"），
gyro 相应给上。纯静止的合成数据一般无法让 LIO 初始化。

`_bias_x` / `_bias_yaw_deg` 用来把点云整体平移/旋转，制造"与地图不匹配"的输入，
便于在定位模式下触发丢失。

用法:
  rosrun rosiwit_slam_ros1 synthetic_publisher.py \
      _imu_topic:=/livox/imu _lidar_topic:=/livox/lidar _rate_lidar:=10
"""
import math
import struct

import rospy
from sensor_msgs.msg import Imu, PointCloud2, PointField


def make_room_points(half=5.0, step=0.25, z_min=-1.0, z_max=2.0):
    """一个封闭房间: 四面墙 + 地面。返回 [(x,y,z,intensity), ...]"""
    pts = []
    z = z_min
    while z <= z_max + 1e-9:
        xy = -half
        while xy <= half + 1e-9:
            pts.append((0.0, xy, z, 100.0))       # 墙: x = 0 平面? 这里用 y-z 墙
            pts.append((xy, 0.0, z, 100.0))       # 墙: x-z 墙
            xy += step
        z += step
    # 地面
    x = -half
    while x <= half + 1e-9:
        y = -half
        while y <= half + 1e-9:
            pts.append((x, y, z_min, 100.0))
            y += step
        x += step
    # 四面墙（±half），让房间封闭
    for side in (1.0, -1.0):
        z = z_min
        while z <= z_max + 1e-9:
            xy = -half
            while xy <= half + 1e-9:
                pts.append((xy, side * half, z, 100.0))
                pts.append((side * half, xy, z, 100.0))
                xy += step
            z += step
    return pts


def main():
    rospy.init_node('synthetic_publisher', anonymous=True)

    imu_topic = rospy.get_param('~imu_topic', '/livox/imu')
    lidar_topic = rospy.get_param('~lidar_topic', '/livox/lidar')
    # 逗号分隔可发布到多个话题 —— 机器人上 legacy profile 到底订 /livox/* 还是 vanjee_*
    # 取决于 sensor_config.json + /machine/model, 一次发到全部候选最稳。
    imu_topics = [t for t in imu_topic.split(',') if t]
    lidar_topics = [t for t in lidar_topic.split(',') if t]
    rate_imu = float(rospy.get_param('~rate_imu', 100.0))
    rate_lidar = float(rospy.get_param('~rate_lidar', 10.0))
    yaw_rate_deg = float(rospy.get_param('~yaw_rate_deg', 11.5))   # 约 0.2 rad/s
    bias_x = float(rospy.get_param('~bias_x', 0.0))
    bias_y = float(rospy.get_param('~bias_y', 0.0))
    bias_yaw_deg = float(rospy.get_param('~bias_yaw_deg', 0.0))
    half = float(rospy.get_param('~room_half', 5.0))
    # 点间距必须**远小于**栅格分辨率, 否则墙会变成点虚线、被 denoise 当噪点清掉。
    # map_resolution=0.05 时用 0.05; 分辨率 0.3 时可放宽到 0.1。
    step = float(rospy.get_param('~step', 0.05))

    yaw_rate = math.radians(yaw_rate_deg)
    bias_yaw = math.radians(bias_yaw_deg)

    imu_pub = [rospy.Publisher(t, Imu, queue_size=200) for t in imu_topics]
    lidar_pub = [rospy.Publisher(t, PointCloud2, queue_size=10) for t in lidar_topics]

    room = make_room_points(half=half, step=step)

    # PointCloud2 布局: x, y, z, intensity (均为 float32)
    fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
        PointField('intensity', 12, PointField.FLOAT32, 1),
    ]

    t0 = rospy.Time.now()

    def publish_lidar(_evt=None):
        now = rospy.Time.now()
        yaw = yaw_rate * (now - t0).to_sec()          # 机器人自转
        c, s = math.cos(yaw), math.sin(yaw)
        cb, sb = math.cos(bias_yaw), math.sin(bias_yaw)
        buf = bytearray()
        for (x, y, z, i) in room:
            xb = cb * x - sb * y + bias_x
            yb = sb * x + cb * y + bias_y
            # 转到雷达本体系: 反向自转
            xr = c * xb + s * yb
            yr = -s * xb + c * yb
            buf += struct.pack('<ffff', xr, yr, z, i)
        msg = PointCloud2()
        msg.header.stamp = now
        msg.header.frame_id = 'livox_frame'
        msg.height = 1
        msg.width = len(room)
        msg.fields = fields
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = 16 * len(room)
        msg.is_dense = True
        msg.data = bytes(buf)
        for p in lidar_pub:
            p.publish(msg)

    def publish_imu(_evt=None):
        now = rospy.Time.now()
        msg = Imu()
        msg.header.stamp = now
        msg.header.frame_id = 'livox_frame'
        # 静止 + 匀速自转: 加速度计读到反作用力(z 向上 9.81), gyro = 自转角速度
        msg.linear_acceleration.x = 0.0
        msg.linear_acceleration.y = 0.0
        msg.linear_acceleration.z = 9.81
        msg.angular_velocity.x = 0.0
        msg.angular_velocity.y = 0.0
        msg.angular_velocity.z = yaw_rate
        for p in imu_pub:
            p.publish(msg)

    rospy.Timer(rospy.Duration(1.0 / rate_lidar), publish_lidar)
    rospy.Timer(rospy.Duration(1.0 / rate_imu), publish_imu)

    rospy.loginfo('synthetic_publisher: imu=%s(%.0fHz) lidar=%s(%.0fHz) points=%d '
                  'yaw_rate=%.1fdeg/s bias=(%.2f,%.2f,%.1fdeg)',
                  imu_topic, rate_imu, lidar_topic, rate_lidar, len(room),
                  yaw_rate_deg, bias_x, bias_y, bias_yaw_deg)
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
