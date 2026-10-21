"""Ground Texture Mapper (ROS2 thin interface).

Subscribes to AVM bird's-eye view images and FAST-LIO2 odometry, feeds them
into the ROS-agnostic ground-texture core, publishes the growing texture map,
and exposes persistence + relocalization services.

Topics in:
  /avm/birdseye       - sensor_msgs/Image (800x800, 5mm/pixel, base_link frame)
  /lio_odom           - nav_msgs/Odometry (robot pose in odom frame)

Topics out:
  /ground_texture_map   - sensor_msgs/Image (global ground texture)
  /ground_texture_grid  - nav_msgs/OccupancyGrid (RViz Map display)
  /ground_texture_pose  - geometry_msgs/PoseStamped (texture-refined pose)
"""

import math
import os

import numpy as np
import rclpy
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import PoseStamped

from rosiwit_perception.ground_texture import GroundTextureMap
from rosiwit_perception_msgs.srv import LoadTextureMap, Relocalize, SaveTextureMap

from .image_codec import bgr_to_imgmsg, imgmsg_to_bgr
from .module import PerceptionModule

# AVM BEV geometry (must match rosiwit_perception.config)
BEV_RESOLUTION = 0.005      # meters per pixel
BEV_SIZE = 800              # BEV image size (800x800)


class GroundTextureMapper(PerceptionModule):
    def __init__(self):
        super().__init__('ground_texture_mapper', module_name='ground_texture')

        bev_resolution = float(self.declare_parameter(
            'bev_resolution', BEV_RESOLUTION).value)
        bev_size = int(self.declare_parameter('bev_size', BEV_SIZE).value)
        ground_z = float(self.declare_parameter('ground_z', -0.11).value)
        initial_canvas = int(self.declare_parameter(
            'initial_canvas_size', 2000).value)
        expand_step = int(self.declare_parameter(
            'canvas_expand_step', 1000).value)
        self.map_frame = self.declare_parameter('map_frame', 'odom').value
        self.save_path = os.path.expanduser(self.declare_parameter(
            'save_path', '~/.ros/rosiwit_ground_texture.npz').value)
        self.auto_load = bool(self.declare_parameter('auto_load', False).value)
        self.relocalize_radius = float(self.declare_parameter(
            'relocalize_search_radius', 0.5).value)
        self.relocalize_angle = float(self.declare_parameter(
            'relocalize_search_angle', 0.3).value)

        self.bridge = None
        self.latest_bev = None
        self.latest_bev_stamp = None
        self._last_integrated_stamp = None

        # 时间戳位姿缓冲区 (用于插值同步)
        self.pose_buffer = []  # [(timestamp_sec, x, y, yaw), ...]
        self.max_pose_buffer = 500

        self.gt = GroundTextureMap(
            bev_resolution=bev_resolution,
            bev_size=bev_size,
            initial_canvas_size=initial_canvas,
            canvas_expand_step=expand_step,
            ground_z=ground_z,
        )
        if self.auto_load:
            self._load_map(self.save_path, quiet=True)

        # Subscribers
        self.create_subscription(Image, '/avm/birdseye', self._bev_callback, 10)
        self.create_subscription(Odometry, '/lio_odom', self._odom_callback, 10)

        # Publishers
        self.map_pub = self.create_publisher(Image, '/ground_texture_map', 10)
        self.grid_pub = self.create_publisher(
            OccupancyGrid, '/ground_texture_grid', 10)
        self.pose_pub = self.create_publisher(
            PoseStamped, '/ground_texture_pose', 10)

        # Services
        self.create_service(
            SaveTextureMap, '~/save_texture_map', self._save_callback)
        self.create_service(
            LoadTextureMap, '~/load_texture_map', self._load_callback)
        self.create_service(
            Relocalize, '~/relocalize', self._relocalize_callback)

        # Publish timer (2 Hz)
        self.publish_timer = self.create_timer(0.5, self._publish_map)

        self.get_logger().info(
            f'GroundTextureMapper initialized: BEV {bev_size}px @ '
            f'{bev_resolution}m/px, canvas {self.gt.canvas_size}px, '
            f'save_path={self.save_path}')

    # ------------------------------------------------------------------ #
    # 输入回调
    # ------------------------------------------------------------------ #
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
            self.latest_bev = imgmsg_to_bgr(msg)
            self.latest_bev_stamp = (
                msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
        except Exception as e:
            self.note_drop('bev_decode')
            self.get_logger().warn(f'BEV decode failed: {e}', once=True)

    def _get_pose_at(self, timestamp):
        """在位姿缓冲区中插值获取指定时间戳的位姿。"""
        if len(self.pose_buffer) < 2:
            return None

        stamps = [p[0] for p in self.pose_buffer]
        idx = np.searchsorted(stamps, timestamp)

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

        dyaw = yaw1 - yaw0
        if dyaw > math.pi:
            dyaw -= 2 * math.pi
        elif dyaw < -math.pi:
            dyaw += 2 * math.pi
        yaw = yaw0 + alpha * dyaw
        return (x, y, yaw)

    # ------------------------------------------------------------------ #
    # 主流程
    # ------------------------------------------------------------------ #
    def _stitch_frame(self):
        if self.latest_bev is None or self.latest_bev_stamp is None:
            return
        if self.latest_bev_stamp == self._last_integrated_stamp:
            return
        self._last_integrated_stamp = self.latest_bev_stamp

        pose = self._get_pose_at(self.latest_bev_stamp)
        if pose is None:
            return

        x, y, yaw = pose
        x, y, yaw = self.gt.integrate(self.latest_bev, x, y, yaw)
        self.note_frame()
        self._publish_pose(x, y, yaw)

        if self.gt.frame_count % 50 == 0:
            self.get_logger().info(
                f'Stitched {self.gt.frame_count} frames, '
                f'pos=({x:.2f},{y:.2f}) yaw={math.degrees(yaw):.1f}deg')

    def _publish_pose(self, x, y, yaw):
        msg = PoseStamped()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = 0.0
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        self.pose_pub.publish(msg)

    def _publish_map(self):
        self._stitch_frame()

        if self.map_pub.get_subscription_count() > 0:
            cropped, _ = self.gt.crop_texture(pad=50)
            if cropped is not None:
                msg = bgr_to_imgmsg(
                    cropped, header=self._header(self.map_frame))
                self.map_pub.publish(msg)

        if self.grid_pub.get_subscription_count() > 0:
            self._publish_grid()

    def _publish_grid(self):
        grid_data = self.gt.occupancy_grid(pad=50)
        if grid_data is None:
            return

        data, origin_x, origin_y, width, height = grid_data

        grid = OccupancyGrid()
        grid.header.frame_id = self.map_frame
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.info.resolution = self.gt.bev_resolution
        grid.info.width = width
        grid.info.height = height
        grid.info.origin.position.x = origin_x
        grid.info.origin.position.y = origin_y
        grid.info.origin.position.z = self.gt.ground_z
        grid.info.origin.orientation.w = 1.0
        grid.data = data.astype(np.int8).flatten().tolist()

        self.grid_pub.publish(grid)

    def _header(self, frame_id):
        h = Header()
        h.frame_id = frame_id
        h.stamp = self.get_clock().now().to_msg()
        return h

    # ------------------------------------------------------------------ #
    # 服务
    # ------------------------------------------------------------------ #
    def _save_callback(self, request, response):
        path = request.path or self.save_path
        try:
            saved = self.gt.save(path)
            self.save_path = saved
            response.success = True
            response.message = f'saved {self.gt.frame_count} frames to {saved}'
            self.get_logger().info(response.message)
        except Exception as exc:
            response.success = False
            response.message = f'save failed: {exc}'
            self.get_logger().error(response.message)
        return response

    def _load_callback(self, request, response):
        path = request.path or self.save_path
        ok, message = self._load_map(path)
        response.success = ok
        response.message = message
        return response

    def _load_map(self, path, quiet=False):
        path = os.path.expanduser(path)
        if not os.path.exists(path):
            if not quiet:
                self.get_logger().warn(f'Texture map not found: {path}')
            return False, f'not found: {path}'
        try:
            self.gt = GroundTextureMap.load(path)
            self.save_path = path
            message = (f'loaded {self.gt.frame_count} frames from {path}, '
                       f'canvas {self.gt.canvas_size}px')
            self.get_logger().info(message)
            return True, message
        except Exception as exc:
            message = f'load failed: {exc}'
            self.get_logger().error(message)
            return False, message

    def _relocalize_callback(self, request, response):
        if self.latest_bev is None:
            response.success = False
            response.message = 'no AVM frame received yet'
            return response

        prior = request.prior_pose
        q = prior.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        x = prior.pose.position.x
        y = prior.pose.position.y

        radius = (request.search_radius if request.search_radius > 0.0
                  else self.relocalize_radius)
        angle = (request.search_angle if request.search_angle > 0.0
                 else self.relocalize_angle)

        ok, x, y, yaw, score = self.gt.localize(
            self.latest_bev, x, y, yaw,
            search_radius=radius, search_angle=angle)

        response.success = bool(ok)
        response.score = float(score)
        response.pose = PoseStamped()
        response.pose.header.frame_id = prior.header.frame_id or self.map_frame
        response.pose.header.stamp = self.get_clock().now().to_msg()
        response.pose.pose.position.x = float(x)
        response.pose.pose.position.y = float(y)
        response.pose.pose.orientation.z = math.sin(yaw / 2.0)
        response.pose.pose.orientation.w = math.cos(yaw / 2.0)
        response.message = (
            f'{"ok" if ok else "low confidence"}: score={score:.3f}, '
            f'search=±{radius:.2f}m/±{math.degrees(angle):.1f}deg')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = GroundTextureMapper()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
