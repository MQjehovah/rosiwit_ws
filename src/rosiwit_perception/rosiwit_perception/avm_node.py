import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np
import cv2
import message_filters

from .config import CAMERA_CONFIGS
from .birdseye import precompute_maps, precompute_maps_with_intrinsics, stitch


class AVMNode(Node):
    def __init__(self):
        super().__init__('avm_node')
        self.bridge = CvBridge()

        self.use_test_pattern = self.declare_parameter('use_test_pattern', False).value

        self.intrinsics = {}
        self.intrinsics_received = 0
        self.maps = None
        self.sync_ready = False

        # CameraInfo 订阅 (一次性，获取内参)
        for cam_name, cfg in CAMERA_CONFIGS.items():
            self.intrinsics[cam_name] = None
            self.create_subscription(
                CameraInfo, cfg['info_topic'],
                lambda msg, name=cam_name: self._info_callback(msg, name),
                10)

        # 发布器
        self.pub = self.create_publisher(Image, '/avm/birdseye', 10)
        self.debug_pubs = {}
        for cam_name in CAMERA_CONFIGS:
            self.debug_pubs[cam_name] = self.create_publisher(
                Image, f'/avm/debug_{cam_name}', 10)

        if self.use_test_pattern:
            self.get_logger().info('Using synthetic test patterns')
            self.maps = precompute_maps()
            self._test_frames = {}
            for cam_name in CAMERA_CONFIGS:
                self._test_frames[cam_name] = self._create_test_pattern(cam_name)
            self.timer = self.create_timer(1.0, self._publish_test_bev)
        else:
            # 延迟创建同步订阅，等内参就绪后再启动
            self.timer = self.create_timer(0.5, self._check_sync_start)

        self.get_logger().info('AVM node initialized, waiting for camera_info...')

    def _check_sync_start(self):
        """等内参就绪后创建时间同步订阅"""
        if self.maps is None or self.sync_ready:
            return
        self.sync_ready = True
        self.timer.cancel()

        # 用 ApproximateTimeSynchronizer 同步4路相机图像
        topics = [CAMERA_CONFIGS[n]['topic'] for n in ['front', 'right', 'back', 'left']]
        subs = []
        for topic in topics:
            sub = message_filters.Subscriber(self, Image, topic)
            subs.append(sub)

        self.sync = message_filters.ApproximateTimeSynchronizer(
            subs, queue_size=10, slop=0.03)  # 30ms 容差
        self.sync.registerCallback(self._synced_camera_callback)
        self.get_logger().info(f'Time-synchronized 4 cameras (slop=30ms)')

    def _info_callback(self, msg, cam_name):
        if self.intrinsics[cam_name] is not None:
            return
        k = msg.k
        self.intrinsics[cam_name] = (k[0], k[4], k[2], k[5])
        self.intrinsics_received += 1
        self.get_logger().info(
            f'{cam_name} camera_info: fx={k[0]:.2f} fy={k[4]:.2f} cx={k[2]:.1f} cy={k[5]:.1f}')

        if self.intrinsics_received == len(CAMERA_CONFIGS) and self.maps is None:
            fx, fy, cx, cy = self.intrinsics['front']
            self.get_logger().info('Recomputing BEV maps with Gazebo intrinsics...')
            self.maps = precompute_maps_with_intrinsics(fx, fy, cx, cy)
            self.get_logger().info('BEV maps ready')

    def _synced_camera_callback(self, msg_front, msg_right, msg_back, msg_left):
        """4路相机时间同步回调 — 所有图像来自同一时刻"""
        if self.maps is None:
            return

        try:
            frames = {}
            for cam_name, msg in [('front', msg_front), ('right', msg_right),
                                   ('back', msg_back), ('left', msg_left)]:
                frames[cam_name] = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

            # 用最早的时间戳作为 BEV 时间戳
            stamp = msg_front.header.stamp

            # Debug: 各相机单独发布
            import cv2 as _cv2
            for cam_name, m in self.maps.items():
                img = frames.get(cam_name)
                if img is None:
                    continue
                warped = _cv2.remap(img, m['map_x'], m['map_y'],
                                    _cv2.INTER_LINEAR, borderMode=_cv2.BORDER_CONSTANT,
                                    borderValue=0)
                mask_3ch = _cv2.merge([m['mask'], m['mask'], m['mask']])
                warped_masked = _cv2.bitwise_and(warped, mask_3ch)
                dbg_msg = self.bridge.cv2_to_imgmsg(warped_masked, 'bgr8')
                dbg_msg.header.frame_id = 'base_link'
                dbg_msg.header.stamp = stamp
                self.debug_pubs[cam_name].publish(dbg_msg)

            # 拼接鸟瞰图
            bev = stitch(frames, self.maps)
            bev_msg = self.bridge.cv2_to_imgmsg(bev, 'bgr8')
            bev_msg.header.frame_id = 'base_link'
            bev_msg.header.stamp = stamp
            self.pub.publish(bev_msg)

        except Exception as e:
            self.get_logger().error(f'Stitched BEV failed: {e}')

    def _create_test_pattern(self, cam_name):
        w, h = 1280, 720
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
                    cv2.rectangle(img, (x, y), (x + cell - 1, y + cell - 1), lighter, -1)
        cv2.putText(img, cam_name.upper(), (w // 2 - 100, h // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 3)
        return img

    def _publish_test_bev(self):
        if self.maps is None:
            return
        bev = stitch(self._test_frames, self.maps)
        msg = self.bridge.cv2_to_imgmsg(bev, 'bgr8')
        msg.header.frame_id = 'base_link'
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AVMNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
