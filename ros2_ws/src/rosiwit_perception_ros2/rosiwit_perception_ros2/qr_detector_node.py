"""QR Code Detector (ROS2 thin interface).

订阅相机图像，调用 ROS 无关核心做检测/解码/位姿估计，并提供：

- ``/qr/detections``  : std_msgs/String (JSON，兼容旧接口)
- ``/qr/codes``       : rosiwit_perception_msgs/QrCodeArray（含 map 系位姿）
- ``/qr/debug_image`` : 标注图
- ``/qr/approach``    : ApproachQr action（视觉伺服靠近二维码）

多帧确认、TF 变换和视觉伺服都在这一层完成，核心算法保持无 ROS 依赖。
"""

import json
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.time import Duration
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from rosiwit_perception.qr_detector import (
    QRDetector,
    compute_approach_cmd,
    is_optical_frame,
    matrix_to_quaternion,
    optical_pose_to_body,
)
from rosiwit_perception_msgs.action import ApproachQr
from rosiwit_perception_msgs.msg import QrCode, QrCodeArray

from .image_codec import bgr_to_imgmsg, imgmsg_to_bgr
from .module import PerceptionModule


class QRDetectorNode(PerceptionModule):
    def __init__(self):
        super().__init__('qr_detector', module_name='qr_detector')

        self.image_topic = self.declare_parameter(
            'image_topic', '/camera_front/camera/image_raw').value
        self.camera_info_topic = self.declare_parameter(
            'camera_info_topic', '/camera_front/camera/camera_info').value
        self.detections_topic = self.declare_parameter(
            'detections_topic', '/qr/detections').value
        self.codes_topic = self.declare_parameter(
            'codes_topic', '/qr/codes').value
        self.debug_image_topic = self.declare_parameter(
            'debug_image_topic', '/qr/debug_image').value

        self.qr_size = float(self.declare_parameter('qr_size', 0.10).value)
        self.min_size_px = float(self.declare_parameter('min_size_px', 24.0).value)
        self.max_rate_hz = float(self.declare_parameter('max_rate_hz', 10.0).value)
        self.publish_empty = bool(
            self.declare_parameter('publish_empty', False).value)
        self.publish_debug_image = bool(
            self.declare_parameter('publish_debug_image', True).value)

        # 多帧确认
        self.confirm_hits = int(self.declare_parameter('confirm_hits', 2).value)
        self.confirm_max_age = float(
            self.declare_parameter('confirm_max_age', 0.5).value)

        # TF / map 位姿
        self.target_frame = self.declare_parameter('target_frame', 'map').value
        self.use_tf = bool(self.declare_parameter('use_tf', True).value)
        self.tf_timeout = float(self.declare_parameter('tf_timeout', 0.05).value)

        # 视觉伺服
        self.action_name = self.declare_parameter(
            'action_name', '/qr/approach').value
        self.cmd_vel_topic = self.declare_parameter(
            'cmd_vel_topic', '/cmd_vel').value
        self.approach_k_linear = float(
            self.declare_parameter('approach_k_linear', 0.8).value)
        self.approach_k_angular = float(
            self.declare_parameter('approach_k_angular', 1.2).value)
        self.approach_max_linear = float(
            self.declare_parameter('approach_max_linear', 0.15).value)
        self.approach_max_angular = float(
            self.declare_parameter('approach_max_angular', 0.5).value)
        self.approach_bearing_gate = float(
            self.declare_parameter('approach_bearing_gate', 0.5).value)
        self.approach_distance_tol = float(
            self.declare_parameter('approach_distance_tol', 0.03).value)
        self.approach_bearing_tol = float(
            self.declare_parameter('approach_bearing_tol', 0.03).value)
        self.approach_loss_timeout = float(
            self.declare_parameter('approach_loss_timeout', 0.8).value)
        self.default_target_distance = float(
            self.declare_parameter('default_target_distance', 0.6).value)
        self.default_timeout = float(
            self.declare_parameter('default_timeout', 30.0).value)

        if self.max_rate_hz > 0.0:
            self._min_period = 1.0 / self.max_rate_hz
        else:
            self._min_period = 0.0
        self._last_process_time = 0.0
        self._last_texts = None

        self.detector = QRDetector(min_size_px=self.min_size_px)

        self.camera_matrix = None
        self.dist_coeffs = None
        self.camera_frame = ''

        # 多帧确认状态: text -> {hits, last_seen, det}
        self._confirmed = {}
        self._state_lock = threading.Lock()
        self._approach_active = False

        self._cb_group = ReentrantCallbackGroup()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.det_pub = self.create_publisher(String, self.detections_topic, 10)
        self.codes_pub = self.create_publisher(QrCodeArray, self.codes_topic, 10)
        self.debug_pub = self.create_publisher(Image, self.debug_image_topic, 10)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        self.create_subscription(
            CameraInfo, self.camera_info_topic, self._info_callback, 10,
            callback_group=self._cb_group)
        self.create_subscription(
            Image, self.image_topic, self._image_callback, 10,
            callback_group=self._cb_group)

        self._action_server = ActionServer(
            self, ApproachQr, self.action_name,
            execute_callback=self._execute_approach,
            goal_callback=self._goal_callback,
            cancel_callback=lambda _goal: CancelResponse.ACCEPT,
            callback_group=self._cb_group,
        )

        self.get_logger().info(
            f'QRDetector listening on {self.image_topic} '
            f'(qr_size={self.qr_size:.3f}m, max_rate={self.max_rate_hz:.1f}Hz, '
            f'target_frame={self.target_frame})')
        if self.qr_size <= 0.0:
            self.get_logger().warn('qr_size <= 0, 3D pose estimation disabled')

    # ------------------------------------------------------------------ #
    # 相机内参
    # ------------------------------------------------------------------ #
    def _info_callback(self, msg: CameraInfo):
        if self.camera_matrix is not None:
            return
        k = msg.k
        self.camera_matrix = np.asarray(k, dtype=np.float64).reshape(3, 3)
        if len(msg.d) > 0:
            self.dist_coeffs = np.asarray(msg.d, dtype=np.float64)
        self.camera_frame = msg.header.frame_id
        self.get_logger().info(
            f'camera_info: fx={k[0]:.2f} fy={k[4]:.2f} '
            f'cx={k[2]:.1f} cy={k[5]:.1f} frame={self.camera_frame or "unknown"}')

    # ------------------------------------------------------------------ #
    # 图像回调
    # ------------------------------------------------------------------ #
    def _image_callback(self, msg: Image):
        now = time.monotonic()
        if self._min_period > 0.0 and now - self._last_process_time < self._min_period:
            return
        self._last_process_time = now

        try:
            image = imgmsg_to_bgr(msg)
        except Exception as exc:
            self.note_drop(f'image_decode:{msg.encoding}')
            self.get_logger().warn(
                f'Image decode failed ({msg.encoding}): {exc}', once=True)
            return

        detections = self.detector.detect(
            image,
            camera_matrix=self.camera_matrix,
            dist_coeffs=self.dist_coeffs,
            marker_size=self.qr_size,
        )

        confirmed = self._update_confirmation(detections)
        self.note_frame(raw=len(detections), confirmed=len(confirmed))

        texts = [d.text for d in confirmed]
        if texts != self._last_texts:
            if texts:
                self.get_logger().info(f'QR detected: {texts}')
            elif self._last_texts:
                self.get_logger().info('No QR code in view')
            self._last_texts = texts

        poses = [self._pose_for(det, msg) for det in confirmed]

        if confirmed or self.publish_empty:
            self.det_pub.publish(String(
                data=self._to_json(msg, confirmed, poses)))
            self.codes_pub.publish(self._to_msg(msg, confirmed, poses))

        if (self.publish_debug_image
                and self.debug_pub.get_subscription_count() > 0):
            self.debug_pub.publish(bgr_to_imgmsg(
                self._draw_debug(image, confirmed), header=msg.header))

    def _update_confirmation(self, detections):
        now = time.monotonic()
        with self._state_lock:
            for det in detections:
                entry = self._confirmed.get(det.text)
                if entry is None or now - entry['last_seen'] > self.confirm_max_age:
                    entry = {'hits': 0, 'last_seen': now, 'det': det}
                entry['hits'] += 1
                entry['last_seen'] = now
                entry['det'] = det
                self._confirmed[det.text] = entry

            stale = [text for text, entry in self._confirmed.items()
                     if now - entry['last_seen'] > self.confirm_max_age]
            for text in stale:
                del self._confirmed[text]

            return [entry['det'] for entry in self._confirmed.values()
                    if entry['hits'] >= self.confirm_hits]

    # ------------------------------------------------------------------ #
    # 位姿 / 消息转换
    # ------------------------------------------------------------------ #
    def _pose_for(self, det, msg: Image):
        """返回二维码位姿 PoseStamped；优先变换到 target_frame。"""
        if det.tvec is None or det.rvec is None:
            return None

        source, rvec, tvec, rotation = self._camera_frame_pose(det, msg)
        if not source:
            return None

        target = self.target_frame
        if self.use_tf and target:
            transform = self._lookup_transform(target, source, msg)
            if transform is not None:
                q, tr = transform
                r_ts = self._quat_to_matrix(q)
                r_sm = cv2.Rodrigues(np.asarray(rvec).reshape(3, 1))[0]
                r_tm = r_ts @ r_sm
                t_tm = r_ts @ np.asarray(tvec) + np.asarray(tr)
                tvec = tuple(float(v) for v in t_tm)
                rotation = matrix_to_quaternion(r_tm)
                source = target
            # TF 不可用时保留相机系位姿，避免整条链路失效

        position = tuple(float(v) for v in tvec)
        pose = PoseStamped()
        pose.header.stamp = msg.header.stamp
        pose.header.frame_id = source
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = position
        (pose.pose.orientation.x, pose.pose.orientation.y,
         pose.pose.orientation.z, pose.pose.orientation.w) = (
            float(rotation[0]), float(rotation[1]),
            float(rotation[2]), float(rotation[3]))
        return pose

    def _camera_frame_pose(self, det, msg: Image):
        """把检测位姿表示到 CameraInfo 的 frame 下。

        Returns ``(frame, rvec, tvec, quat)``。
        """
        source = self.camera_frame or msg.header.frame_id
        if not source or det.tvec is None or det.rvec is None:
            return '', None, None, None

        rvec = det.rvec
        tvec = det.tvec
        if not is_optical_frame(source):
            rvec, tvec = optical_pose_to_body(rvec, tvec)

        rotation = matrix_to_quaternion(
            cv2.Rodrigues(np.asarray(rvec).reshape(3, 1))[0])
        return source, rvec, tvec, rotation

    def _lookup_transform(self, target, source, msg):
        try:
            tf = self.tf_buffer.lookup_transform(
                target, source, msg.header.stamp,
                timeout=Duration(seconds=self.tf_timeout))
        except Exception:
            try:
                tf = self.tf_buffer.lookup_transform(target, source, rclpy.time.Time())
            except Exception as exc:
                self.get_logger().warn(
                    f'TF {target} <- {source} unavailable: {exc}',
                    throttle_duration_sec=5.0)
                return None
        t = tf.transform.translation
        q = tf.transform.rotation
        return (q.x, q.y, q.z, q.w), (t.x, t.y, t.z)

    @staticmethod
    def _quat_to_matrix(q):
        x, y, z, w = q
        norm = np.sqrt(x * x + y * y + z * z + w * w)
        if norm <= 0.0:
            return np.eye(3)
        x, y, z, w = x / norm, y / norm, z / norm, w / norm
        return np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ])

    def _to_msg(self, msg: Image, detections, poses) -> QrCodeArray:
        arr = QrCodeArray()
        arr.header = msg.header
        for det, pose in zip(detections, poses):
            code = QrCode()
            code.text = det.text
            code.corners = [float(v) for v in det.corners.reshape(-1)]
            code.center = [float(det.center[0]), float(det.center[1])]
            code.size = float(self.qr_size)
            code.distance = float(det.distance) if det.distance else 0.0
            if pose is not None:
                code.pose = pose
            arr.codes.append(code)
        return arr

    def _to_json(self, msg: Image, detections, poses) -> str:
        items = []
        for det, pose in zip(detections, poses):
            item = {
                'text': det.text,
                'corners': [[round(float(x), 2), round(float(y), 2)]
                            for x, y in det.corners],
                'center': [round(float(det.center[0]), 2),
                           round(float(det.center[1]), 2)],
            }
            source, _, tvec_cam, quat_cam = self._camera_frame_pose(det, msg)
            if tvec_cam is not None:
                item['pose'] = {
                    'frame_id': source,
                    'position': {
                        'x': round(float(tvec_cam[0]), 5),
                        'y': round(float(tvec_cam[1]), 5),
                        'z': round(float(tvec_cam[2]), 5),
                    },
                    'orientation': {
                        'x': round(float(quat_cam[0]), 6),
                        'y': round(float(quat_cam[1]), 6),
                        'z': round(float(quat_cam[2]), 6),
                        'w': round(float(quat_cam[3]), 6),
                    },
                    'distance': round(float(det.distance), 5),
                }
            if pose is not None:
                item['pose_target'] = {
                    'frame_id': pose.header.frame_id,
                    'position': {
                        'x': pose.pose.position.x,
                        'y': pose.pose.position.y,
                        'z': pose.pose.position.z,
                    },
                    'orientation': {
                        'x': pose.pose.orientation.x,
                        'y': pose.pose.orientation.y,
                        'z': pose.pose.orientation.z,
                        'w': pose.pose.orientation.w,
                    },
                }
            items.append(item)

        payload = {
            'header': {
                'stamp': {
                    'sec': int(msg.header.stamp.sec),
                    'nanosec': int(msg.header.stamp.nanosec),
                },
                'frame_id': msg.header.frame_id,
            },
            'count': len(items),
            'detections': items,
        }
        return json.dumps(payload, ensure_ascii=False)

    # ------------------------------------------------------------------ #
    # 视觉伺服 action
    # ------------------------------------------------------------------ #
    def _goal_callback(self, goal_request):
        if self._approach_active:
            self.get_logger().warn('Reject approach goal: another goal is active')
            return GoalResponse.REJECT
        if not goal_request.text:
            self.get_logger().warn('Reject approach goal: empty text')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _execute_approach(self, goal_handle):
        goal = goal_handle.request
        result = ApproachQr.Result()
        feedback = ApproachQr.Feedback()

        target_distance = (goal.target_distance
                           if goal.target_distance > 0.0
                           else self.default_target_distance)
        timeout = goal.timeout if goal.timeout > 0.0 else self.default_timeout

        self._approach_active = True
        start = time.monotonic()
        last_seen = start
        last_distance = 0.0
        last_bearing = 0.0

        self.get_logger().info(
            f'Approach QR "{goal.text}" -> {target_distance:.2f}m, '
            f'timeout={timeout:.1f}s')

        try:
            while rclpy.ok():
                now = time.monotonic()
                elapsed = now - start

                if goal_handle.is_cancel_requested:
                    self._stop_base()
                    goal_handle.canceled()
                    result.success = False
                    result.message = 'canceled'
                    return result

                if elapsed > timeout:
                    self._stop_base()
                    goal_handle.abort()
                    result.success = False
                    result.message = f'timeout after {elapsed:.1f}s'
                    return result

                with self._state_lock:
                    entry = self._confirmed.get(goal.text)
                    det = entry['det'] if entry is not None else None

                if det is None or det.tvec is None:
                    if now - last_seen > self.approach_loss_timeout:
                        self._stop_base()
                        goal_handle.abort()
                        result.success = False
                        result.message = f'QR "{goal.text}" lost'
                        return result
                    self._stop_base()
                    time.sleep(0.05)
                    continue

                last_seen = now
                x, _, z = (float(v) for v in det.tvec)
                if z <= 1e-6:
                    self._stop_base()
                    goal_handle.abort()
                    result.success = False
                    result.message = 'QR behind camera'
                    return result

                distance = float(np.linalg.norm(det.tvec))
                bearing = float(np.arctan2(x, z))
                last_distance = distance
                last_bearing = bearing

                linear, angular, done = compute_approach_cmd(
                    distance, bearing, target_distance,
                    k_linear=self.approach_k_linear,
                    k_angular=self.approach_k_angular,
                    max_linear=self.approach_max_linear,
                    max_angular=self.approach_max_angular,
                    bearing_gate=self.approach_bearing_gate,
                    distance_tol=self.approach_distance_tol,
                    bearing_tol=self.approach_bearing_tol,
                )

                feedback.distance = distance
                feedback.bearing = bearing
                feedback.elapsed = elapsed
                goal_handle.publish_feedback(feedback)

                if done and goal_handle.is_active:
                    self._stop_base()
                    goal_handle.succeed()
                    result.success = True
                    result.message = (
                        f'reached {distance:.2f}m (target {target_distance:.2f}m)')
                    return result

                cmd = Twist()
                cmd.linear.x = linear
                cmd.angular.z = angular
                self.cmd_pub.publish(cmd)
                time.sleep(0.05)

            self._stop_base()
            result.success = False
            result.message = 'ros shutdown'
            return result
        finally:
            self._stop_base()
            self._approach_active = False
            self.get_logger().info(
                f'Approach finished: distance={last_distance:.2f}m '
                f'bearing={last_bearing:.2f}rad')

    def _stop_base(self):
        self.cmd_pub.publish(Twist())

    # ------------------------------------------------------------------ #
    # 调试图像
    # ------------------------------------------------------------------ #
    def _draw_debug(self, image, detections):
        debug = image.copy()
        for det in detections:
            pts = det.corners.astype(int).reshape(-1, 1, 2)
            cv2.polylines(debug, [pts], True, (0, 255, 0), 2)
            center = tuple(int(v) for v in det.center)
            cv2.circle(debug, center, 4, (0, 0, 255), -1)

            label = det.text
            if det.distance is not None:
                label += f'  {det.distance:.2f}m'
            anchor = (int(det.corners[0][0]), int(det.corners[0][1]) - 10)
            cv2.putText(debug, label, anchor, cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (0, 255, 0), 2, cv2.LINE_AA)

            if det.rvec is not None and self.camera_matrix is not None:
                try:
                    cv2.drawFrameAxes(
                        debug,
                        self.camera_matrix,
                        self.dist_coeffs if self.dist_coeffs is not None else [],
                        det.rvec,
                        det.tvec,
                        max(self.qr_size * 0.5, 0.01),
                        2,
                    )
                except cv2.error:
                    pass
        return debug


def main(args=None):
    rclpy.init(args=args)
    node = QRDetectorNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
