"""360° AVM (Around View Monitor) node.

四路相机 -> BEV 鸟瞰图。相比旧版：

- 每路相机使用自己的内参/畸变系数（不再共用 front 内参）
- 支持从 TF 读取外参（``use_tf_extrinsics``）
- 按相机缓存最新帧，单路掉线时可用剩余相机降级拼接
- 统一心跳，接入 ``/perception/status`` 与 ``/diagnostics``
"""

import time

import cv2
import numpy as np
import rclpy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener

from rosiwit_perception.config import (
    CAMERA_CONFIGS, FX, FY, CX, CY, IMG_W, IMG_H,
)
from rosiwit_perception.birdseye import (
    precompute_maps,
    precompute_maps_from_models,
    stitch,
)

from .image_codec import bgr_to_imgmsg, imgmsg_to_bgr
from .module import PerceptionModule


class AVMNode(PerceptionModule):
    def __init__(self):
        super().__init__('avm_node', module_name='avm')

        self.use_test_pattern = bool(
            self.declare_parameter('use_test_pattern', False).value)
        self.stitch_rate = float(
            self.declare_parameter('stitch_rate', 15.0).value)
        self.sync_slop = float(
            self.declare_parameter('sync_slop', 0.03).value)
        self.camera_timeout = float(
            self.declare_parameter('camera_timeout', 1.0).value)
        self.min_cameras = int(
            self.declare_parameter('min_cameras', 3).value)
        self.use_tf_extrinsics = bool(
            self.declare_parameter('use_tf_extrinsics', False).value)

        self.intrinsics = {name: None for name in CAMERA_CONFIGS}
        self.intrinsics_received = 0
        self.maps = None

        # cam_name -> (stamp_sec, recv_monotonic, bgr)
        self.frames = {}
        self._degraded = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # CameraInfo 订阅 (一次性，获取内参/畸变)
        for cam_name, cfg in CAMERA_CONFIGS.items():
            self.create_subscription(
                CameraInfo, cfg['info_topic'],
                lambda msg, name=cam_name: self._info_callback(msg, name),
                10)
            self.create_subscription(
                Image, cfg['topic'],
                lambda msg, name=cam_name: self._image_callback(msg, name),
                10)

        # 发布器
        self.pub = self.create_publisher(Image, '/avm/birdseye', 10)
        self.debug_pubs = {
            cam_name: self.create_publisher(
                Image, f'/avm/debug_{cam_name}', 10)
            for cam_name in CAMERA_CONFIGS
        }

        if self.use_test_pattern:
            self.get_logger().info('Using synthetic test patterns')
            self.maps = precompute_maps()
            self._test_frames = {
                cam_name: self._create_test_pattern(cam_name)
                for cam_name in CAMERA_CONFIGS
            }
            self.create_timer(1.0, self._publish_test_bev)
        else:
            self.create_timer(1.0 / max(self.stitch_rate, 1.0), self._stitch_timer)

        self.get_logger().info(
            f'AVM node initialized (stitch={self.stitch_rate:.1f}Hz, '
            f'min_cameras={self.min_cameras}, tf_extrinsics={self.use_tf_extrinsics})')

    # ------------------------------------------------------------------ #
    # 标定
    # ------------------------------------------------------------------ #
    def _info_callback(self, msg: CameraInfo, cam_name):
        if self.intrinsics[cam_name] is not None:
            return
        k = msg.k
        fx, fy, cx, cy = k[0], k[4], k[2], k[5]
        if fx <= 0.0 or fy <= 0.0:
            fx, fy, cx, cy = FX, FY, CX, CY
            self.get_logger().warn(
                f'{cam_name} has invalid intrinsics, using defaults')

        self.intrinsics[cam_name] = {
            'fx': fx, 'fy': fy, 'cx': cx, 'cy': cy,
            'dist': list(msg.d) if len(msg.d) > 0 else None,
        }
        self.intrinsics_received += 1
        self.get_logger().info(
            f'{cam_name} camera_info: fx={fx:.2f} fy={fy:.2f} '
            f'cx={cx:.1f} cy={cy:.1f} dist={len(msg.d)}')

        if self.intrinsics_received == len(CAMERA_CONFIGS) and self.maps is None:
            self._build_maps()

    def _build_maps(self):
        models = {}
        for cam_name, cfg in CAMERA_CONFIGS.items():
            intr = self.intrinsics[cam_name]
            if intr is None:
                return
            models[cam_name] = dict(intr, R=cfg['R'], t=cfg['t'])

        tf_transforms = None
        if self.use_tf_extrinsics:
            tf_transforms = {}
            for cam_name, cfg in CAMERA_CONFIGS.items():
                frame = cfg.get('frame')
                if not frame:
                    continue
                try:
                    tf = self.tf_buffer.lookup_transform(
                        'base_link', frame, Time())
                    tf_transforms[cam_name] = tf.transform
                except Exception as exc:
                    self.get_logger().warn(
                        f'TF base_link <- {frame} unavailable, '
                        f'using config extrinsic: {exc}')

        self.maps = precompute_maps_from_models(
            models, tf_transforms or None)
        self.get_logger().info('BEV maps ready (per-camera intrinsics/distortion)')

    # ------------------------------------------------------------------ #
    # 图像
    # ------------------------------------------------------------------ #
    def _image_callback(self, msg: Image, cam_name):
        try:
            image = imgmsg_to_bgr(msg)
        except Exception as exc:
            self.note_drop(f'{cam_name}_decode')
            self.get_logger().warn(
                f'{cam_name} decode failed ({msg.encoding}): {exc}', once=True)
            return

        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.frames[cam_name] = (stamp, time.monotonic(), image)

    def _stitch_timer(self):
        if self.maps is None:
            return

        now = time.monotonic()
        available = {}
        for cam_name, (stamp, recv, image) in self.frames.items():
            if now - recv <= self.camera_timeout:
                available[cam_name] = (stamp, image)

        if len(available) < max(1, self.min_cameras):
            self.note_drop('missing_cameras')
            return

        stamps = [v[0] for v in available.values()]
        if max(stamps) - min(stamps) > self.sync_slop:
            self.note_drop('sync')
            return

        frames = {name: value[1] for name, value in available.items()}
        stamp_sec = max(v[0] for v in available.values())
        sec = int(stamp_sec)
        nanosec = int(round((stamp_sec - sec) * 1e9))
        stamp_msg = Time(seconds=sec, nanoseconds=nanosec).to_msg()

        try:
            bev = stitch(frames, self.maps)
            self.pub.publish(bgr_to_imgmsg(
                bev, header=self._header('base_link', stamp_msg)))

            for cam_name, m in self.maps.items():
                img = frames.get(cam_name)
                if img is None:
                    continue
                warped = cv2.remap(
                    img, m['map_x'], m['map_y'], cv2.INTER_LINEAR,
                    borderMode=cv2.BORDER_CONSTANT, borderValue=0)
                mask_3ch = cv2.merge([m['mask'], m['mask'], m['mask']])
                dbg = bgr_to_imgmsg(
                    cv2.bitwise_and(warped, mask_3ch),
                    header=self._header('base_link', stamp_msg))
                self.debug_pubs[cam_name].publish(dbg)

            degraded = len(frames) < len(CAMERA_CONFIGS)
            if degraded != self._degraded:
                self.get_logger().warn(
                    f'AVM degraded mode: {len(frames)}/{len(CAMERA_CONFIGS)} cameras')
                self._degraded = degraded
            self.note_frame(cameras=len(frames), degraded=degraded)
        except Exception as exc:
            self.note_drop('stitch_error')
            self.get_logger().error(f'Stitched BEV failed: {exc}')

    def _header(self, frame_id, stamp):
        header = Header()
        header.frame_id = frame_id
        header.stamp = stamp
        return header

    def _publish_test_bev(self):
        if self.maps is None:
            return
        bev = stitch(self._test_frames, self.maps)
        msg = bgr_to_imgmsg(
            bev, header=self._header('base_link',
                                     self.get_clock().now().to_msg()))
        self.pub.publish(msg)
        self.note_frame(cameras=4, degraded=False)

    def _create_test_pattern(self, cam_name):
        w, h = IMG_W, IMG_H
        img = np.zeros((h, w, 3), dtype=np.uint8)
        colors = {'front': (180, 0, 0), 'right': (0, 180, 0),
                  'back': (0, 0, 180), 'left': (180, 180, 0)}
        color = colors.get(cam_name, (128, 128, 128))
        img[:] = color
        cell = 40
        lighter = tuple(min(c + 75, 255) for c in color)
        for y in range(0, h, cell):
            for x in range(0, w, cell):
                if (x // cell + y // cell) % 2 == 0:
                    cv2.rectangle(img, (x, y),
                                  (x + cell - 1, y + cell - 1), lighter, -1)
        cv2.putText(img, cam_name.upper(), (w // 2 - 100, h // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 3)
        return img


def main(args=None):
    rclpy.init(args=args)
    node = AVMNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
