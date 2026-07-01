import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np
import cv2

from .config import CAMERA_CONFIGS
from .birdseye import precompute_maps, precompute_maps_with_intrinsics, stitch


class AVMNode(Node):
    def __init__(self):
        super().__init__('avm_node')
        self.bridge = CvBridge()

        self.use_test_pattern = self.declare_parameter('use_test_pattern', False).value

        self.frames = {}
        self.frame_stamps = {}
        self.intrinsics = {}
        self.intrinsics_received = 0
        self.maps = None

        for cam_name, cfg in CAMERA_CONFIGS.items():
            topic = cfg['topic']
            info_topic = cfg['info_topic']
            self.frames[cam_name] = None
            self.frame_stamps[cam_name] = None
            self.intrinsics[cam_name] = None
            self.create_subscription(
                Image, topic,
                lambda msg, name=cam_name: self._camera_callback(msg, name),
                10)
            self.create_subscription(
                CameraInfo, info_topic,
                lambda msg, name=cam_name: self._info_callback(msg, name),
                10)
            self.get_logger().info(f'Subscribed to {topic} and {info_topic}')

        self.pub = self.create_publisher(Image, '/avm/birdseye', 10)
        # Debug: publish each camera's warped view separately
        self.debug_pubs = {}
        for cam_name in CAMERA_CONFIGS:
            self.debug_pubs[cam_name] = self.create_publisher(
                Image, f'/avm/debug_{cam_name}', 10)

        if self.use_test_pattern:
            self.get_logger().info('Using synthetic test patterns')
            self.maps = precompute_maps()
            for cam_name in CAMERA_CONFIGS:
                self.frames[cam_name] = self._create_test_pattern(cam_name)
            self.timer = self.create_timer(1.0, self._publish_bev)
        else:
            self.timer = self.create_timer(0.05, self._publish_bev)

        self.get_logger().info('AVM node initialized, waiting for camera_info...')

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
            self.get_logger().info(f'Recomputing BEV maps with Gazebo intrinsics...')
            self.maps = precompute_maps_with_intrinsics(fx, fy, cx, cy)
            self.get_logger().info('BEV maps ready')

    def _camera_callback(self, msg, cam_name):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.frames[cam_name] = cv_img
            self.frame_stamps[cam_name] = msg.header.stamp
        except Exception as e:
            self.get_logger().warn(f'Failed to decode {cam_name}: {e}')

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

    def _publish_bev(self):
        if self.maps is None:
            return
        ready = sum(1 for v in self.frames.values() if v is not None)
        if ready < 4:
            return
        try:
            from .config import BEV_WIDTH, BEV_HEIGHT, BEV_RESOLUTION
            import cv2 as _cv2

            # Publish each camera's individual warped view for debugging
            for cam_name, m in self.maps.items():
                img = self.frames.get(cam_name)
                if img is None:
                    continue
                warped = _cv2.remap(img, m['map_x'], m['map_y'],
                                    _cv2.INTER_LINEAR, borderMode=_cv2.BORDER_CONSTANT,
                                    borderValue=0)
                # Apply mask: black out invalid pixels
                mask_3ch = _cv2.merge([m['mask'], m['mask'], m['mask']])
                warped_masked = _cv2.bitwise_and(warped, mask_3ch)
                msg = self.bridge.cv2_to_imgmsg(warped_masked, 'bgr8')
                msg.header.frame_id = 'base_link'
                msg.header.stamp = self.get_clock().now().to_msg()
                self.debug_pubs[cam_name].publish(msg)

            bev = stitch(self.frames, self.maps)
            msg = self.bridge.cv2_to_imgmsg(bev, 'bgr8')
            msg.header.frame_id = 'base_link'
            msg.header.stamp = self.get_clock().now().to_msg()
            self.pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Stitching failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = AVMNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
