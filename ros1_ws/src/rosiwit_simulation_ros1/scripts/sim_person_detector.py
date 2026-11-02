#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""仿真用"完美"2D 检测器。

从 Gazebo 的 /gazebo/model_states 读取行人模型的真值位姿，按相机外参投影成
2D 框，发布 ObjectDetection2DArray。仅用于仿真链路验证，不参与产品运行时。
"""
import math

import rospy
import tf.transformations as tft
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import CameraInfo, Image

from rosiwit_perception_ros1.msg import ObjectDetection2D, ObjectDetection2DArray


def parse_floats(text, n, default):
    vals = [float(x) for x in text.replace(';', ',').split(',') if x.strip()]
    if len(vals) != n:
        return list(default)
    return vals


class SimPersonDetector(object):
    def __init__(self):
        self.robot_name = rospy.get_param('~robot_name', 'mrobot')
        names = rospy.get_param('~model_names', 'person_1,person_2,person_3')
        self.model_names = [n.strip() for n in names.split(',') if n.strip()]
        self.model_size = parse_floats(rospy.get_param('~model_size', '0.5,0.3,1.7'), 3,
                                       (0.5, 0.3, 1.7))
        self.camera_offset = parse_floats(rospy.get_param('~camera_offset', '0.525,0,0.6725'), 3,
                                          (0.525, 0.0, 0.6725))
        self.class_id = int(rospy.get_param('~class_id', 0))
        self.class_name = rospy.get_param('~class_name', 'person')
        self.score = float(rospy.get_param('~score', 0.95))
        self.max_range = float(rospy.get_param('~max_range', 8.0))

        self.color_topic = rospy.get_param('~color_topic', '/camera_01/color/image_raw')
        self.info_topic = rospy.get_param('~camera_info_topic', '/camera_01/color/camera_info')
        self.states_topic = rospy.get_param('~model_states_topic', '/gazebo/model_states')
        self.output_topic = rospy.get_param('~output_topic', '/perception/objects/detections_2d')

        # base_link <- camera_optical（x 前/y 左/z 上 <- x 右/y 下/z 前）
        self.R_base_cam = ((0.0, 0.0, 1.0),
                           (-1.0, 0.0, 0.0),
                           (0.0, -1.0, 0.0))

        self.cam_info = None
        self.states = None
        self.pub = rospy.Publisher(self.output_topic, ObjectDetection2DArray, queue_size=2)
        rospy.Subscriber(self.info_topic, CameraInfo, self.info_cb, queue_size=1)
        rospy.Subscriber(self.states_topic, ModelStates, self.states_cb, queue_size=1)
        rospy.Subscriber(self.color_topic, Image, self.image_cb, queue_size=1)
        rospy.loginfo('[sim_person_detector] models=%s -> %s',
                      ','.join(self.model_names), self.output_topic)

    def info_cb(self, msg):
        self.cam_info = msg

    def states_cb(self, msg):
        self.states = msg

    @staticmethod
    def quat_to_rot(q):
        m = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
        return ((m[0][0], m[0][1], m[0][2]),
                (m[1][0], m[1][1], m[1][2]),
                (m[2][0], m[2][1], m[2][2]))

    @staticmethod
    def mat_mul(a, b):
        return tuple(tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3))
                     for i in range(3))

    @staticmethod
    def mat_vec(a, v):
        return tuple(sum(a[i][k] * v[k] for k in range(3)) for i in range(3))

    @staticmethod
    def mat_t_vec(a, v):
        return tuple(sum(a[k][i] * v[k] for k in range(3)) for i in range(3))

    def image_cb(self, image):
        if self.cam_info is None or self.states is None:
            return
        # 世界模型里可能有同名 mrobot（如 simple.world），取 z 最高的那个
        robot_idxs = [i for i, n in enumerate(self.states.name) if n == self.robot_name]
        if not robot_idxs:
            return
        robot_idx = max(robot_idxs, key=lambda i: self.states.pose[i].position.z)
        rp = self.states.pose[robot_idx]
        R_wb = self.quat_to_rot(rp.orientation)
        t_wb = (rp.position.x, rp.position.y, rp.position.z)
        R_wc = self.mat_mul(R_wb, self.R_base_cam)
        t_bc = self.mat_vec(R_wb, self.camera_offset)
        t_wc = (t_wb[0] + t_bc[0], t_wb[1] + t_bc[1], t_wb[2] + t_bc[2])

        K = self.cam_info.K
        fx, fy, cx, cy = K[0], K[4], K[2], K[5]
        width = self.cam_info.width
        height = self.cam_info.height
        hw = self.model_size[0] * 0.5
        hd = self.model_size[1] * 0.5
        mh = self.model_size[2]

        out = ObjectDetection2DArray()
        out.header.stamp = image.header.stamp
        out.header.frame_id = image.header.frame_id

        for name in self.model_names:
            try:
                idx = self.states.name.index(name)
            except ValueError:
                continue
            pose = self.states.pose[idx]
            R_wp = self.quat_to_rot(pose.orientation)
            tp = (pose.position.x, pose.position.y, pose.position.z)
            u_min, u_max = 1e9, -1e9
            v_min, v_max = 1e9, -1e9
            ok = True
            for sx in (-hw, hw):
                for sy in (-hd, hd):
                    for sz in (0.0, mh):
                        local = self.mat_vec(R_wp, (sx, sy, sz))
                        pw = (tp[0] + local[0], tp[1] + local[1], tp[2] + local[2])
                        d = (pw[0] - t_wc[0], pw[1] - t_wc[1], pw[2] - t_wc[2])
                        pc = self.mat_t_vec(R_wc, d)
                        if pc[2] < 0.15:
                            ok = False
                            break
                        u = fx * pc[0] / pc[2] + cx
                        v = fy * pc[1] / pc[2] + cy
                        u_min, u_max = min(u_min, u), max(u_max, u)
                        v_min, v_max = min(v_min, v), max(v_max, v)
                    if not ok:
                        break
                if not ok:
                    break
            if not ok:
                continue

            u0 = max(0.0, u_min)
            v0 = max(0.0, v_min)
            u1 = min(float(width - 1), u_max)
            v1 = min(float(height - 1), v_max)
            if u1 - u0 < 4.0 or v1 - v0 < 8.0:
                continue
            # 距离超限/完全在画面外
            cxw = pose.position.x
            cyw = pose.position.y
            if math.hypot(cxw - t_wc[0], cyw - t_wc[1]) > self.max_range:
                continue

            det = ObjectDetection2D()
            det.header = out.header
            det.class_id = self.class_id
            det.class_name = self.class_name
            det.score = self.score
            det.x_min, det.y_min = u0, v0
            det.x_max, det.y_max = u1, v1
            out.detections.append(det)

        self.pub.publish(out)


if __name__ == '__main__':
    rospy.init_node('sim_person_detector')
    SimPersonDetector()
    rospy.spin()
