"""
Ground Texture Mapper (ROS2 thin interface)

Subscribes to AVM bird's-eye view images and FAST-LIO2 odometry, then feeds
them into the ROS-agnostic ground-texture core (rosiwit_perception.GroundTextureMap),
publishing the growing global texture map.

Topics in:
  /avm/birdseye       - sensor_msgs/Image (800x800, 5mm/pixel, base_link frame)
  /lio_odom           - nav_msgs/Odometry (robot pose in odom frame)

Topics out:
  /ground_texture_map  - sensor_msgs/Image (global ground texture)
  /ground_texture_grid - nav_msgs/OccupancyGrid (for RViz Map display)
"""

import bisect
import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry, OccupancyGrid
from cv_bridge import CvBridge

from rosiwit_perception.ground_texture import GroundTextureMap

# AVM BEV geometry (must match rosiwit_perception.config)
BEV_RESOLUTION = 0.005      # meters per pixel
BEV_SIZE = 800              # BEV image size (800x800)


class GroundTextureMapper(Node):
    def __init__(self):
        super().__init__('ground_texture_mapper')

        self.bridge = CvBridge()
        self.latest_bev = None
        self.latest_bev_stamp = None

        # 时间戳位姿缓冲区 (用于插值同步)
        self.pose_buffer = []  # [(timestamp_sec, x, y, yaw), ...]
        self.max_pose_buffer = 500

        # ROS 无关核心: 全局纹理拼接器
        self.gt = GroundTextureMap(
            bev_resolution=BEV_RESOLUTION,
            bev_size=BEV_SIZE,
            initial_canvas_size=2000,
            canvas_expand_step=1000,
            ground_z=-0.11,
        )

        # Subscribers
        self.create_subscription(Image, '/avm/birdseye', self._bev_callback, 10)
        self.create_subscription(Odometry, '/lio_odom', self._odom_callback, 10)

        # Publishers
        self.map_pub = self.create_publisher(Image, '/ground_texture_map', 10)
        self.grid_pub = self.create_publisher(OccupancyGrid, '/ground_texture_grid', 10)

        # Publish timer (2 Hz)
        self.publish_timer = self.create_timer(0.5, self._publish_map)

        self.get_logger().info(
            f'GroundTextureMapper initialized: BEV {BEV_SIZE}px @ '
            f'{BEV_RESOLUTION}m/px, canvas {self.gt.canvas_size}px')

    def _odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        self.pose_buffer.append((stamp, x, y, yaw))
        if len(self.pose_buffer) > self.max_pose_buffer:
            self.pose_buffer.pop(0)

    def _bev_callback(self, msg):
        try:
            self.latest_bev = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.latest_bev_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        except Exception as e:
            self.get_logger().warn(f'BEV decode failed: {e}')

    def _get_pose_at(self, timestamp):
        """在位姿缓冲区中插值获取指定时间戳的位姿"""
        if len(self.pose_buffer) < 2:
            return None

        stamps = [p[0] for p in self.pose_buffer]
        idx = bisect.bisect_left(stamps, timestamp)

        if idx == 0:
            return self.pose_buffer[0][1:]
        if idx >= len(self.pose_buffer):
            return self.pose_buffer[-1][1:]

        t0, x0, y0, yaw0 = self.pose_buffer[idx - 1]
        t1, x1, y1, yaw1 = self.pose_buffer[idx]
        if t1 == t0:
            return (x0, y0, yaw0)

        alpha = max(0.0, min(1.0, (timestamp - t0) / (t1 - t0)))
        x = x0 + alpha * (x1 - x0)
        y = y0 + alpha * (y1 - y0)

        # 角度插值 (处理 wrap-around)
        dyaw = yaw1 - yaw0
        if dyaw > math.pi:
            dyaw -= 2 * math.pi
        elif dyaw < -math.pi:
            dyaw += 2 * math.pi
        yaw = yaw0 + alpha * dyaw

        return (x, y, yaw)

    def _stitch_frame(self):
        if self.latest_bev is None or self.latest_bev_stamp is None:
            return

        # 用 AVM 图像时间戳插值获取对应位姿 (解决运动中不同步)
        pose = self._get_pose_at(self.latest_bev_stamp)
        if pose is None:
            return

        x, y, yaw = pose

        # 核心: 位姿精化 + 融合
        x, y, yaw = self.gt.integrate(self.latest_bev, x, y, yaw)

        if self.gt.frame_count % 50 == 0:
            self.get_logger().info(
                f'Stitched {self.gt.frame_count} frames, '
                f'pos=({x:.2f},{y:.2f}) yaw={math.degrees(yaw):.1f}deg')

    def _publish_map(self):
        self._stitch_frame()

        if self.map_pub.get_subscription_count() > 0:
            cropped, _ = self.gt.crop_texture(pad=50)
            if cropped is not None:
                msg = self.bridge.cv2_to_imgmsg(cropped, 'bgr8')
                msg.header.frame_id = 'odom'
                msg.header.stamp = self.get_clock().now().to_msg()
                self.map_pub.publish(msg)

        if self.grid_pub.get_subscription_count() > 0:
            self._publish_grid()

    def _publish_grid(self):
        """Publish as OccupancyGrid for RViz Map display."""
        grid_data = self.gt.occupancy_grid(pad=50)
        if grid_data is None:
            return

        data, origin_x, origin_y, width, height = grid_data

        grid = OccupancyGrid()
        grid.header.frame_id = 'odom'
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.info.resolution = BEV_RESOLUTION
        grid.info.width = width
        grid.info.height = height
        grid.info.origin.position.x = origin_x
        grid.info.origin.position.y = origin_y
        grid.info.origin.position.z = self.gt.ground_z
        grid.info.origin.orientation.w = 1.0
        grid.data = data.astype(np.int8).flatten().tolist()

        self.grid_pub.publish(grid)


def main(args=None):
    rclpy.init(args=args)
    node = GroundTextureMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
