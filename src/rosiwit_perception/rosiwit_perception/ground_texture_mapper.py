"""
Ground Texture Mapper

Subscribes to AVM bird's-eye view images and FAST-LIO2 odometry,
stitches ground textures into a growing global map using alpha blending.

Topics in:
  /avm/birdseye   - sensor_msgs/Image (800x800, 5mm/pixel, base_link frame)
  /lio_odom       - nav_msgs/Odometry (robot pose in odom frame)

Topics out:
  /ground_texture_map - sensor_msgs/Image (global ground texture)
  /ground_texture_grid - nav_msgs/OccupancyGrid (for RViz Map display)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry, OccupancyGrid
from cv_bridge import CvBridge
import numpy as np
import cv2
import math


BEV_RESOLUTION = 0.005      # meters per pixel (matches AVM config)
BEV_SIZE = 800              # BEV image size (800x800)
BEV_COVERAGE = BEV_SIZE * BEV_RESOLUTION  # 4.0 meters
BEV_HALF = BEV_COVERAGE / 2.0

INITIAL_CANVAS_SIZE = 2000  # initial canvas pixels (10m x 10m at 5mm/px)
CANVAS_EXPAND_STEP = 1000   # pixels to expand when robot nears edge


class GroundTextureMapper(Node):
    def __init__(self):
        super().__init__('ground_texture_mapper')

        self.bridge = CvBridge()
        self.latest_bev = None
        self.latest_pose = None
        self.frame_count = 0

        # Canvas: growing global texture map
        # Canvas coordinate: pixel (0,0) at top-left
        # World coordinate: origin (0,0) at canvas center initially
        self.canvas_size = INITIAL_CANVAS_SIZE
        self.canvas_center_x = self.canvas_size // 2
        self.canvas_center_y = self.canvas_size // 2
        self.texture_canvas = np.zeros(
            (self.canvas_size, self.canvas_size, 3), dtype=np.uint8)
        self.weight_canvas = np.zeros(
            (self.canvas_size, self.canvas_size), dtype=np.float32)

        # Subscribers
        self.create_subscription(
            Image, '/avm/birdseye', self._bev_callback, 10)
        self.create_subscription(
            Odometry, '/lio_odom', self._odom_callback, 10)

        # Publishers
        self.map_pub = self.create_publisher(
            Image, '/ground_texture_map', 10)
        self.grid_pub = self.create_publisher(
            OccupancyGrid, '/ground_texture_grid', 10)

        # Publish timer (2 Hz)
        self.publish_timer = self.create_timer(0.5, self._publish_map)

        self.get_logger().info(
            f'GroundTextureMapper initialized: '
            f'BEV {BEV_SIZE}px @ {BEV_RESOLUTION}m/px, '
            f'canvas {self.canvas_size}px')

    def _odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        # Extract yaw from quaternion
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.latest_pose = (x, y, yaw)

    def _bev_callback(self, msg):
        try:
            self.latest_bev = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn(f'BEV decode failed: {e}')

    def _refine_pose(self, bev, x, y, yaw):
        """
        Refine pose using template matching against existing canvas.
        Corrects small LIO drift by aligning the new BEV with the existing texture.
        Returns corrected (x, y, yaw).
        """
        if self.frame_count < 5:
            return x, y, yaw  # not enough canvas yet

        res = BEV_RESOLUTION
        # Search range: ±15cm translation, ±3° rotation
        search_radius = int(0.15 / res)  # 30 pixels
        search_angles = np.linspace(-0.05, 0.05, 7)  # ±2.9°
        bcx_val = BEV_SIZE / 2.0

        # Extract a small BEV patch (center 300x300) for matching
        patch_r = 150
        bc = BEV_SIZE // 2
        template = cv2.cvtColor(bev[bc-patch_r:bc+patch_r, bc-patch_r:bc+patch_r], cv2.COLOR_BGR2GRAY)

        best_score = -1
        best_dx = 0
        best_dy = 0
        best_dyaw = 0

        for d_yaw in search_angles:
            test_yaw = yaw + d_yaw
            sy = math.sin(test_yaw)
            cy = math.cos(test_yaw)
            cx_canvas = float(self.canvas_center_x)
            cy_canvas = float(self.canvas_center_y)

            T_x = bcx_val * (cy - sy) + x / res + cx_canvas
            T_y = -bcx_val * (sy + cy) - y / res + cy_canvas

            M = np.array([[sy, -cy, T_x], [cy, sy, T_y]], dtype=np.float32)

            # Warp BEV to canvas
            warped = cv2.warpAffine(
                bev, M, (self.canvas_size, self.canvas_size),
                flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=0)

            # Extract search region around predicted center
            px = int(x / res + cx_canvas)
            py = int(-y / res + cy_canvas)
            search_r = patch_r + search_radius

            x0 = max(0, px - search_r)
            y0 = max(0, py - search_r)
            x1 = min(self.canvas_size, px + search_r)
            y1 = min(self.canvas_size, py + search_r)

            if x1 - x0 < patch_r * 2 or y1 - y0 < patch_r * 2:
                continue

            search_region = cv2.cvtColor(
                self.texture_canvas[y0:y1, x0:x1], cv2.COLOR_BGR2GRAY)
            warped_region = cv2.cvtColor(
                warped[y0:y1, x0:x1], cv2.COLOR_BGR2GRAY)

            # Only match where both have content
            mask = (warped_region > 10) & (search_region > 10)
            if mask.sum() < patch_r * patch_r:
                continue

            try:
                result = cv2.matchTemplate(
                    search_region, template, cv2.TM_CCOEFF_NORMED)
                _, max_val, _, max_loc = cv2.minMaxLoc(result)
                if max_val > best_score:
                    best_score = max_val
                    best_dx = max_loc[0] - (patch_r + search_radius - patch_r)
                    best_dy = max_loc[1] - (patch_r + search_radius - patch_r)
                    best_dyaw = d_yaw
            except cv2.error:
                continue

        if best_score > 0.3:
            # Convert pixel correction to world coordinates
            dx_world = best_dx * res
            dy_world = -best_dy * res  # flip Y
            x += dx_world
            y += dy_world
            yaw += best_dyaw

        return x, y, yaw

    def _stitch_frame(self):
        if self.latest_bev is None or self.latest_pose is None:
            return

        bev = self.latest_bev
        x, y, yaw = self.latest_pose

        # Refine pose using local template matching (disabled for debugging)
        # x, y, yaw = self._refine_pose(bev, x, y, yaw)

        res = BEV_RESOLUTION
        bcx = bcy = BEV_SIZE / 2.0  # BEV center = 400

        # Robot position in canvas pixels
        px = self.canvas_center_x + int(x / res)
        py = self.canvas_center_y - int(y / res)
        margin = int(BEV_HALF / res) + 10

        # Expand canvas if needed
        if (px - margin < 0 or px + margin >= self.canvas_size or
                py - margin < 0 or py + margin >= self.canvas_size):
            self._expand_canvas()
            px = self.canvas_center_x + int(x / res)
            py = self.canvas_center_y - int(y / res)

        # Affine matrix: BEV pixel (u,v) → canvas pixel (cu, cv)
        #
        # Chain: BEV(u,v) → base_link(x,y) → world(x,y) → canvas(u,v)
        #   x_base = (bcy - v) * res     [top=front]
        #   y_base = (bcx - u) * res     [left=+y]
        #   x_world = cos(yaw)*x_base - sin(yaw)*y_base + x
        #   y_world = sin(yaw)*x_base + cos(yaw)*y_base + y
        #   cu = x_world/res + cx
        #   cv = -y_world/res + cy
        #
        # Combined:
        #   cu = sin(yaw)*u - cos(yaw)*v + bcx*(cos(yaw)-sin(yaw)) + x/res + cx
        #   cv = cos(yaw)*u + sin(yaw)*v - bcx*(sin(yaw)+cos(yaw)) - y/res + cy

        sy = math.sin(yaw)
        cy = math.cos(yaw)
        cx = float(self.canvas_center_x)
        cy_canvas = float(self.canvas_center_y)

        T_x = bcx * (cy - sy) + x / res + cx
        T_y = -bcx * (sy + cy) - y / res + cy_canvas

        M = np.array([
            [sy, -cy, T_x],
            [cy,  sy, T_y],
        ], dtype=np.float32)

        # Warp full BEV onto full canvas (then blend)
        warped = cv2.warpAffine(
            bev, M,
            (self.canvas_size, self.canvas_size),
            flags=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=0)

        # Weight mask: radial falloff for seamless blending
        weight_full = self._build_weight_mask(BEV_SIZE)
        warped_weight = cv2.warpAffine(
            weight_full, M,
            (self.canvas_size, self.canvas_size),
            flags=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=0)

        # Alpha blend only valid (non-zero) regions
        valid = warped_weight > 0.01
        w_new = warped_weight[valid]
        w_old = self.weight_canvas[valid]
        total_w = w_old + w_new
        safe = np.maximum(total_w, 1e-6)

        old_c = self.texture_canvas[valid].astype(np.float32)
        new_c = warped[valid].astype(np.float32)
        blended = (old_c * w_old[:, None] + new_c * w_new[:, None]) / safe[:, None]

        self.texture_canvas[valid] = blended.astype(np.uint8)
        self.weight_canvas[valid] = np.minimum(total_w, 10.0)

        self.frame_count += 1
        if self.frame_count % 50 == 0:
            self.get_logger().info(
                f'Stitched {self.frame_count} frames, '
                f'pos=({x:.2f},{y:.2f}) yaw={math.degrees(yaw):.1f}deg')

    def _build_weight_mask(self, size):
        """Build radial falloff weight mask for blending."""
        cx = cy = size / 2.0
        y_coords, x_coords = np.mgrid[0:size, 0:size]
        dist = np.sqrt((x_coords - cx) ** 2 + (y_coords - cy) ** 2)
        max_dist = size / 2.0
        # Smooth falloff: 1.0 at center, 0.0 at edge
        weight = 1.0 - np.clip(dist / max_dist, 0, 1) ** 2
        return weight.astype(np.float32)

    def _expand_canvas(self):
        """Double the canvas size, keeping existing content centered."""
        new_size = self.canvas_size + CANVAS_EXPAND_STEP * 2
        offset = CANVAS_EXPAND_STEP

        new_texture = np.zeros(
            (new_size, new_size, 3), dtype=np.uint8)
        new_weight = np.zeros(
            (new_size, new_size), dtype=np.float32)

        new_texture[offset:offset + self.canvas_size,
                    offset:offset + self.canvas_size] = self.texture_canvas
        new_weight[offset:offset + self.canvas_size,
                   offset:offset + self.canvas_size] = self.weight_canvas

        self.texture_canvas = new_texture
        self.weight_canvas = new_weight
        self.canvas_size = new_size
        self.canvas_center_x += offset
        self.canvas_center_y += offset

        self.get_logger().info(
            f'Canvas expanded to {new_size}x{new_size}px')

    def _publish_map(self):
        self._stitch_frame()

        if self.map_pub.get_subscription_count() > 0:
            # Crop to non-zero region for efficiency
            ys, xs = np.where(self.weight_canvas > 0.01)
            if len(ys) == 0:
                return

            pad = 50
            y0 = max(0, ys.min() - pad)
            y1 = min(self.canvas_size, ys.max() + pad)
            x0 = max(0, xs.min() - pad)
            x1 = min(self.canvas_size, xs.max() + pad)

            cropped = self.texture_canvas[y0:y1, x0:x1]
            msg = self.bridge.cv2_to_imgmsg(cropped, 'bgr8')
            msg.header.frame_id = 'odom'
            msg.header.stamp = self.get_clock().now().to_msg()
            self.map_pub.publish(msg)

        if self.grid_pub.get_subscription_count() > 0:
            self._publish_grid()

    def _publish_grid(self):
        """Publish as OccupancyGrid for RViz Map display."""
        ys, xs = np.where(self.weight_canvas > 0.01)
        if len(ys) == 0:
            return

        pad = 50
        y0 = max(0, ys.min() - pad)
        y1 = min(self.canvas_size, ys.max() + pad)
        x0 = max(0, xs.min() - pad)
        x1 = min(self.canvas_size, xs.max() + pad)

        sub_w = self.weight_canvas[y0:y1, x0:x1]
        sub_t = self.texture_canvas[y0:y1, x0:x1]

        # Convert to grayscale occupancy: observed=0(free), unobserved=-1(unknown)
        gray = cv2.cvtColor(sub_t, cv2.COLOR_BGR2GRAY)
        data = np.full_like(gray, -1, dtype=np.int8)
        observed = sub_w > 0.01
        data[observed] = 0  # free space where we have texture

        grid = OccupancyGrid()
        grid.header.frame_id = 'odom'
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.info.resolution = BEV_RESOLUTION
        grid.info.width = int(x1 - x0)
        grid.info.height = int(y1 - y0)

        # Origin: world position of pixel (x0, y0) in canvas
        origin_x = (x0 - self.canvas_center_x) * BEV_RESOLUTION
        origin_y = -(y1 - self.canvas_center_y) * BEV_RESOLUTION  # flip Y
        grid.info.origin.position.x = origin_x
        grid.info.origin.position.y = origin_y
        grid.info.origin.position.z = -0.11  # ground level
        grid.info.origin.orientation.w = 1.0
        grid.data = data.flatten().tolist()

        self.grid_pub.publish(grid)


def main(args=None):
    rclpy.init(args=args)
    node = GroundTextureMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
