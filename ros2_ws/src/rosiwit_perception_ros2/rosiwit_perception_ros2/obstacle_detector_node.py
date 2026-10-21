"""Camera-based low obstacle detector (ROS2 thin interface).

把前视相机中的低矮障碍（纸箱、椅子腿、线缆等 LiDAR 容易漏掉的目标）
分割出来，投影到地面平面，发布 ``sensor_msgs/PointCloud2``（base_link）。

该点云可以直接接入导航的安全层/体素层：
  - ``safety_pointcloud_topic``
  - ``voxel_cloud_topic``
"""

import time

import numpy as np
import rclpy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener

from rosiwit_perception.config import CAMERA_CONFIGS, FX, FY, CX, CY
from rosiwit_perception.birdseye import _R_LINK_TO_OPTICAL, _quat_to_matrix
from rosiwit_perception.obstacle_detector import GroundObstacleDetector

from .image_codec import imgmsg_to_bgr
from .module import PerceptionModule


class ObstacleDetectorNode(PerceptionModule):
    def __init__(self):
        super().__init__('obstacle_detector', module_name='obstacle_detector')

        self.camera_name = self.declare_parameter(
            'camera_name', 'front').value
        if self.camera_name not in CAMERA_CONFIGS:
            raise ValueError(f'unknown camera_name: {self.camera_name}')
        cfg = CAMERA_CONFIGS[self.camera_name]

        self.image_topic = self.declare_parameter(
            'image_topic', cfg['topic']).value
        self.camera_info_topic = self.declare_parameter(
            'camera_info_topic', cfg['info_topic']).value
        self.cloud_topic = self.declare_parameter(
            'cloud_topic', '/perception/obstacles').value
        self.stop_topic = self.declare_parameter(
            'stop_topic', '/perception/obstacle_stop').value
        self.stop_distance = float(self.declare_parameter(
            'stop_distance', 0.35).value)
        self.obstacle_height = float(self.declare_parameter(
            'obstacle_height', 0.10).value)
        self.max_rate_hz = float(self.declare_parameter(
            'max_rate_hz', 10.0).value)
        self.use_tf_extrinsics = bool(self.declare_parameter(
            'use_tf_extrinsics', False).value)

        ground_z = float(self.declare_parameter('ground_z', -0.11).value)
        self.detector = GroundObstacleDetector(
            ground_z=ground_z,
            max_distance=float(self.declare_parameter(
                'max_distance', 3.0).value),
            max_lateral=float(self.declare_parameter(
                'max_lateral', 1.5).value),
            min_area_ratio=float(self.declare_parameter(
                'min_area_ratio', 0.0015).value),
            color_threshold=float(self.declare_parameter(
                'color_threshold', 35.0).value),
            roi_top_ratio=float(self.declare_parameter(
                'roi_top_ratio', 0.30).value),
        )

        self.cfg = cfg
        self.ground_z = ground_z
        self.R = np.asarray(cfg['R'], dtype=np.float64)
        self.t = np.asarray(cfg['t'], dtype=np.float64)
        self.camera_matrix = np.array([
            [FX, 0.0, CX], [0.0, FY, CY], [0.0, 0.0, 1.0]])
        self.dist_coeffs = None

        self._min_period = (1.0 / self.max_rate_hz
                            if self.max_rate_hz > 0.0 else 0.0)
        self._last_process = 0.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._tf_loaded = False
        if self.use_tf_extrinsics:
            self.create_timer(0.5, self._try_load_tf_extrinsics)

        self.cloud_pub = self.create_publisher(
            PointCloud2, self.cloud_topic, 10)
        self.stop_pub = self.create_publisher(Bool, self.stop_topic, 10)
        self.create_subscription(
            CameraInfo, self.camera_info_topic, self._info_callback, 10)
        self.create_subscription(
            Image, self.image_topic, self._image_callback, 10)

        self.get_logger().info(
            f'ObstacleDetector camera={self.camera_name} '
            f'{self.image_topic} -> {self.cloud_topic}')

    def _try_load_tf_extrinsics(self):
        if self._tf_loaded:
            return
        frame = self.cfg.get('frame')
        try:
            tf = self.tf_buffer.lookup_transform('base_link', frame, Time())
        except Exception:
            return
        r_link = _quat_to_matrix([
            tf.transform.rotation.x, tf.transform.rotation.y,
            tf.transform.rotation.z, tf.transform.rotation.w])
        self.R = _R_LINK_TO_OPTICAL @ r_link
        self.t = self.R @ np.array([
            tf.transform.translation.x,
            tf.transform.translation.y,
            tf.transform.translation.z])
        self._tf_loaded = True
        self.get_logger().info(f'Using TF extrinsics from {frame}')

    def _info_callback(self, msg: CameraInfo):
        k = msg.k
        if k[0] > 0.0 and k[4] > 0.0:
            self.camera_matrix = np.asarray(
                k, dtype=np.float64).reshape(3, 3)
        if len(msg.d) > 0:
            self.dist_coeffs = np.asarray(msg.d, dtype=np.float64)

    def _image_callback(self, msg: Image):
        now = time.monotonic()
        if self._min_period > 0.0 and now - self._last_process < self._min_period:
            return
        self._last_process = now

        try:
            image = imgmsg_to_bgr(msg)
        except Exception as exc:
            self.note_drop('image_decode')
            self.get_logger().warn(f'decode failed: {exc}', once=True)
            return

        obstacles = self.detector.detect(
            image, self.camera_matrix, self.dist_coeffs, self.R, self.t)

        points = np.array(
            [[o.x, o.y, self.obstacle_height] for o in obstacles],
            dtype=np.float32)
        self.cloud_pub.publish(self._make_cloud(points, msg.header))
        nearest = min((float(np.hypot(o.x, o.y)) for o in obstacles),
                      default=float('inf'))
        self.stop_pub.publish(Bool(data=nearest < self.stop_distance))
        self.note_frame(obstacles=len(obstacles))

    def _make_cloud(self, points: np.ndarray, header) -> PointCloud2:
        cloud = PointCloud2()
        cloud.header = header
        cloud.header.frame_id = 'base_link'
        cloud.height = 1
        cloud.width = int(points.shape[0])
        cloud.is_dense = True
        cloud.is_bigendian = False
        cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.point_step = 12
        cloud.row_step = cloud.point_step * cloud.width
        cloud.data = np.ascontiguousarray(points, dtype=np.float32).tobytes()
        return cloud


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
