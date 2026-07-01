import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2

from .config import CAMERA_CONFIGS
from .birdseye import precompute_maps, stitch


class AVMNode(Node):
    def __init__(self):
        super().__init__('avm_node')
        self.bridge = CvBridge()

        # If true, use synthetic test images instead of real camera topics
        self.use_test_pattern = self.declare_parameter('use_test_pattern', False).value

        # Initialize frame buffer
        self.frames = {}
        self.frame_stamps = {}

        # Precompute bird's-eye view mapping
        self.get_logger().info('Precomputing AVM remap tables...')
        self.maps = precompute_maps()
        self.get_logger().info('AVM remap tables ready')

        # Subscribe to 4 camera topics
        for cam_name, cfg in CAMERA_CONFIGS.items():
            topic = cfg['topic']
            self.frames[cam_name] = None
            self.frame_stamps[cam_name] = None
            self.create_subscription(
                Image, topic,
                lambda msg, name=cam_name: self._camera_callback(msg, name),
                10)
            self.get_logger().info(f'Subscribed to {topic}')

        # Publisher for bird's-eye view
        self.pub = self.create_publisher(Image, '/avm/birdseye', 10)

        if self.use_test_pattern:
            self.get_logger().info('Using synthetic test patterns (use_test_pattern=True)')
            for cam_name in CAMERA_CONFIGS:
                self.frames[cam_name] = self._create_test_pattern(cam_name)
            self.timer = self.create_timer(1.0, self._publish_bev)
        else:
            self.timer = self.create_timer(0.05, self._publish_bev)  # 20Hz

        self.get_logger().info('AVM node initialized')

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
                    cv2.rectangle(img, (x, y), (x + cell - 1, y + cell - 1),
                                  lighter, -1)

        cv2.putText(img, cam_name.upper(), (w // 2 - 100, h // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 3)

        return img

    def _publish_bev(self):
        # Check which cameras have recent frames
        ready = sum(1 for v in self.frames.values() if v is not None)
        if ready < 4:
            return

        try:
            # Stitch
            bev = stitch(self.frames, self.maps)

            # Publish
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
