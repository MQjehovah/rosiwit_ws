#!/usr/bin/env python3
"""Frontier-based autonomous exploration for ROS 2.

The node consumes a live ``nav_msgs/OccupancyGrid`` map, extracts frontier
clusters (free cells adjacent to unknown space), filters out unreachable or
unsafe candidates, and sends goals through either:

* ``/goal_pose`` (geometry_msgs/PoseStamped) - works with the in-repo
  ``rosiwit_navigation_ros2`` node, or
* the Nav2 ``navigate_to_pose`` action.

No SLAM or simulator code is embedded here; the package only consumes the map
and robot pose and produces navigation goals.
"""

import math
import os
import subprocess
import time
from collections import deque
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformListener, TransformException
from visualization_msgs.msg import Marker, MarkerArray

try:
    from nav2_msgs.action import NavigateToPose
except ImportError:  # pragma: no cover - nav2_msgs is an exec dependency
    NavigateToPose = None

try:
    from tf2_geometry_msgs import do_transform_pose
except ImportError:  # pragma: no cover
    do_transform_pose = None


@dataclass
class FrontierCandidate:
    """A reachable frontier cluster and its chosen navigation goal."""

    x: float
    y: float
    size: int
    distance: float
    score: float
    row: int
    col: int


def dilate_unknown(unknown: np.ndarray) -> np.ndarray:
    """8-connected dilation of the unknown mask without wrap-around."""
    h, w = unknown.shape
    padded = np.pad(unknown, 1, constant_values=False)
    out = np.zeros_like(unknown, dtype=bool)
    for dy in range(3):
        for dx in range(3):
            out |= padded[dy:dy + h, dx:dx + w]
    return out


def extract_clusters(mask: np.ndarray, min_size: int) -> List[List[Tuple[int, int]]]:
    """Return 8-connected clusters of ``mask`` with at least ``min_size`` cells."""
    h, w = mask.shape
    visited = np.zeros_like(mask, dtype=bool)
    clusters: List[List[Tuple[int, int]]] = []

    for row, col in np.argwhere(mask):
        row = int(row)
        col = int(col)
        if visited[row, col]:
            continue
        stack = [(row, col)]
        visited[row, col] = True
        cells: List[Tuple[int, int]] = []
        while stack:
            r, c = stack.pop()
            cells.append((r, c))
            r0, r1 = max(0, r - 1), min(h, r + 2)
            c0, c1 = max(0, c - 1), min(w, c + 2)
            for nr in range(r0, r1):
                for nc in range(c0, c1):
                    if mask[nr, nc] and not visited[nr, nc]:
                        visited[nr, nc] = True
                        stack.append((nr, nc))
        if len(cells) >= min_size:
            clusters.append(cells)
    return clusters


def build_integral(occupied: np.ndarray) -> np.ndarray:
    """Summed-area table for O(1) occupied-window queries."""
    occ = occupied.astype(np.int32)
    integral = np.zeros((occ.shape[0] + 1, occ.shape[1] + 1), dtype=np.int32)
    integral[1:, 1:] = occ.cumsum(axis=0).cumsum(axis=1)
    return integral


def window_sum(integral: np.ndarray, r0: int, c0: int, r1: int, c1: int) -> int:
    """Number of occupied cells in the half-open box [r0, r1) x [c0, c1)."""
    return int(integral[r1, c1] - integral[r0, c1] - integral[r1, c0] + integral[r0, c0])


def reachable_free_mask(free: np.ndarray, start: Tuple[int, int]) -> np.ndarray:
    """Flood fill free space from ``start`` using 4-connectivity."""
    h, w = free.shape
    reachable = np.zeros_like(free, dtype=bool)
    r0, c0 = start
    if not (0 <= r0 < h and 0 <= c0 < w) or not free[r0, c0]:
        return reachable
    queue = deque([(r0, c0)])
    reachable[r0, c0] = True
    while queue:
        r, c = queue.popleft()
        for nr, nc in ((r - 1, c), (r + 1, c), (r, c - 1), (r, c + 1)):
            if 0 <= nr < h and 0 <= nc < w and free[nr, nc] and not reachable[nr, nc]:
                reachable[nr, nc] = True
                queue.append((nr, nc))
    return reachable


def nearest_free_cell(free: np.ndarray, row: int, col: int,
                      radius: int = 6) -> Optional[Tuple[int, int]]:
    """Find a free cell near (row, col), used when the robot cell is unknown."""
    h, w = free.shape
    if 0 <= row < h and 0 <= col < w and free[row, col]:
        return row, col
    best: Optional[Tuple[int, int]] = None
    best_d = 10 ** 9
    for r in range(max(0, row - radius), min(h, row + radius + 1)):
        for c in range(max(0, col - radius), min(w, col + radius + 1)):
            if not free[r, c]:
                continue
            d = (r - row) ** 2 + (c - col) ** 2
            if d < best_d:
                best_d = d
                best = (r, c)
    return best


def yaw_from_points(from_x: float, from_y: float, to_x: float, to_y: float) -> float:
    return math.atan2(to_y - from_y, to_x - from_x)


def quaternion_from_yaw(yaw: float) -> Quaternion:
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw * 0.5), w=math.cos(yaw * 0.5))


def cell_to_world(row: int, col: int, origin_x: float, origin_y: float,
                  resolution: float) -> Tuple[float, float]:
    return (origin_x + (col + 0.5) * resolution,
            origin_y + (row + 0.5) * resolution)


def parse_goal_status(data: str) -> Tuple[str, Optional[int]]:
    """Parse ``status[:goal_stamp_ns]`` published on /goal_status."""
    raw = data.strip()
    status = raw.lower()
    token: Optional[int] = None
    if ':' in raw:
        head, tail = raw.rsplit(':', 1)
        if tail.lstrip('-').isdigit():
            status = head.strip().lower()
            token = int(tail)
    return status, token


def blacklist_duration_for(failures: int, first_failure: float,
                           normal: float) -> float:
    """First failure gets a short retry window; repeated failure is long."""
    return first_failure if failures < 2 else normal


class FrontierExplorer(Node):
    """Explore unknown space by repeatedly driving to the best frontier."""

    def __init__(self) -> None:
        super().__init__('frontier_explorer')

        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('goal_interface', 'pose')  # pose | nav2 | auto
        self.declare_parameter('nav2_action', 'navigate_to_pose')
        self.declare_parameter('global_costmap_topic', '/global_costmap')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('map_frame', '')
        self.declare_parameter('goal_frame', 'odom')
        self.declare_parameter('tick_period', 2.0)

        self.declare_parameter('free_thresh', 50)
        self.declare_parameter('occupied_thresh', 55)
        self.declare_parameter('min_frontier_cells', 8)
        self.declare_parameter('goal_clearance', 0.6)
        self.declare_parameter('goal_backoff', 0.4)
        self.declare_parameter('min_goal_distance', 0.8)
        self.declare_parameter('max_goal_distance', 30.0)
        self.declare_parameter('frontier_size_weight', 0.02)
        self.declare_parameter('nav_cost_weight', 0.5)
        self.declare_parameter('check_reachability', True)

        self.declare_parameter('blacklist_radius', 0.8)
        self.declare_parameter('blacklist_duration', 180.0)
        # 第一次规划失败通常是地图还在增长/局部未知造成的瞬态失败,
        # 短黑名单后允许重试; 同一目标连续失败才进入长黑名单.
        self.declare_parameter('first_failure_blacklist', 10.0)
        self.declare_parameter('goal_tolerance', 0.35)
        self.declare_parameter('goal_timeout', 120.0)
        self.declare_parameter('goal_accept_timeout', 5.0)
        self.declare_parameter('goal_cooldown', 1.0)
        self.declare_parameter('finish_patience', 20)

        self.declare_parameter('autostart', True)
        self.declare_parameter('start_delay', 5.0)
        self.declare_parameter('save_map_on_finish', False)
        self.declare_parameter('save_map_path', '/tmp/rosiwit_exploration_map')
        self.declare_parameter('publish_markers', True)

        self.map_topic = self.get_parameter('map_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.goal_topic = self.get_parameter('goal_topic').value
        self.goal_interface = str(self.get_parameter('goal_interface').value).lower()
        self.nav2_action = self.get_parameter('nav2_action').value
        self.global_costmap_topic = self.get_parameter('global_costmap_topic').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.map_frame_param = self.get_parameter('map_frame').value
        self.goal_frame = self.get_parameter('goal_frame').value

        self.free_thresh = int(self.get_parameter('free_thresh').value)
        self.occupied_thresh = int(self.get_parameter('occupied_thresh').value)
        self.min_frontier_cells = int(self.get_parameter('min_frontier_cells').value)
        self.goal_clearance = float(self.get_parameter('goal_clearance').value)
        self.goal_backoff = float(self.get_parameter('goal_backoff').value)
        self.min_goal_distance = float(self.get_parameter('min_goal_distance').value)
        self.max_goal_distance = float(self.get_parameter('max_goal_distance').value)
        self.frontier_size_weight = float(self.get_parameter('frontier_size_weight').value)
        self.nav_cost_weight = float(self.get_parameter('nav_cost_weight').value)
        self.check_reachability = bool(self.get_parameter('check_reachability').value)

        self.blacklist_radius = float(self.get_parameter('blacklist_radius').value)
        self.blacklist_duration = float(self.get_parameter('blacklist_duration').value)
        self.first_failure_blacklist = float(
            self.get_parameter('first_failure_blacklist').value)
        self.goal_tolerance = float(self.get_parameter('goal_tolerance').value)
        self.goal_timeout = float(self.get_parameter('goal_timeout').value)
        self.goal_accept_timeout = float(self.get_parameter('goal_accept_timeout').value)
        self.goal_cooldown = float(self.get_parameter('goal_cooldown').value)
        self.finish_patience = int(self.get_parameter('finish_patience').value)

        self.autostart = bool(self.get_parameter('autostart').value)
        self.start_delay = float(self.get_parameter('start_delay').value)
        self.save_map_on_finish = bool(self.get_parameter('save_map_on_finish').value)
        self.save_map_path = self.get_parameter('save_map_path').value
        self.publish_markers = bool(self.get_parameter('publish_markers').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.grid: Optional[np.ndarray] = None
        self.map_info = None
        self.map_frame = self.map_frame_param or 'map'
        self.last_odom: Optional[Odometry] = None
        self._last_known_cells: Optional[int] = None

        # 导航节点发布的 /global_costmap: 它已经包含 robot_radius 硬边界和
        # inflation_radius 蓝区. explorer 选目标必须和规划器用同一张图,
        # 否则会出现"地图 free 但 A* 无解".
        self.nav_grid: Optional[np.ndarray] = None
        self.nav_info = None
        self._nav_seq = 0
        self._nav_reachable_cache: Optional[np.ndarray] = None
        self._nav_reachable_key = None

        self.enabled = self.autostart
        self.finished = False
        self.empty_cycles = 0
        self._started_at = time.monotonic()

        self.current_goal: Optional[PoseStamped] = None
        self.current_goal_xy: Optional[Tuple[float, float]] = None
        self.goal_mode = ''
        self.goal_sent_at = 0.0
        self.goal_seq = 0
        self.goal_expected_stamp_ns: Optional[int] = None
        self.goal_accepted = False
        self.nav2_goal_handle = None
        self.next_goal_at = 0.0
        self.blacklist: List[Tuple[float, float, float]] = []
        # 每个前沿目标的连续失败次数, 用于区分瞬态失败和真不可达.
        self.goal_failures: Dict[Tuple[int, int], int] = {}

        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(OccupancyGrid, self.map_topic, self.map_callback, map_qos)
        self.create_subscription(OccupancyGrid, self.global_costmap_topic,
                                 self.global_costmap_callback, map_qos)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)

        self.goal_pub = self.create_publisher(PoseStamped, self.goal_topic, 10)
        self.goal_status_sub = self.create_subscription(
            String, '/goal_status', self.goal_status_cb, 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'frontier_markers', 10)
        self.stop_pub = self.create_publisher(String, '/nav_stop', 10)
        self.stop_client = self.create_client(Trigger, '/stop_navigation')

        self.action_client = None
        if NavigateToPose is not None and self.goal_interface in ('nav2', 'auto'):
            self.action_client = ActionClient(self, NavigateToPose, self.nav2_action)

        self.create_service(SetBool, '~/set_enabled', self.set_enabled_callback)
        self.create_service(Trigger, '~/status', self.status_callback)

        self.create_timer(float(self.get_parameter('tick_period').value), self.tick)

        self.get_logger().info(
            f'Frontier explorer ready: map={self.map_topic} goal={self.goal_topic} '
            f'interface={self.goal_interface} frame={self.goal_frame or "<map>"}'
        )

    # ------------------------------------------------------------------ callbacks
    def map_callback(self, msg: OccupancyGrid) -> None:
        expected = msg.info.width * msg.info.height
        if expected == 0 or len(msg.data) != expected:
            self.get_logger().warning('Ignoring malformed OccupancyGrid')
            return
        self.grid = np.asarray(msg.data, dtype=np.int8).reshape(
            msg.info.height, msg.info.width)
        self.map_info = msg.info
        if not self.map_frame_param and msg.header.frame_id:
            self.map_frame = msg.header.frame_id

        known = int(np.count_nonzero(self.grid >= 0))
        if self._last_known_cells is not None:
            growth = known - self._last_known_cells
            if growth > max(50, int(0.001 * max(known, 1))):
                self.empty_cycles = 0
        self._last_known_cells = known

    def global_costmap_callback(self, msg: OccupancyGrid) -> None:
        expected = msg.info.width * msg.info.height
        if expected == 0 or len(msg.data) != expected:
            self.get_logger().warning('Ignoring malformed global costmap')
            return
        self.nav_grid = np.asarray(msg.data, dtype=np.int16).reshape(
            msg.info.height, msg.info.width)
        self.nav_info = msg.info
        self._nav_seq += 1
        self._nav_reachable_cache = None
        self._nav_reachable_key = None

    def odom_callback(self, msg: Odometry) -> None:
        self.last_odom = msg

    def set_enabled_callback(self, request: SetBool.Request,
                             response: SetBool.Response) -> SetBool.Response:
        self.enabled = bool(request.data)
        if self.enabled:
            self.finished = False
            self.empty_cycles = 0
            self.blacklist.clear()
            self.goal_failures.clear()
            self._started_at = time.monotonic()
            response.message = 'exploration enabled'
        else:
            response.message = 'exploration disabled'
            self._cancel_nav2_goal()
            self.request_stop()
            self.current_goal = None
            self.current_goal_xy = None
            self.goal_accepted = False
            self.goal_expected_stamp_ns = None
        response.success = True
        self.get_logger().info(response.message)
        return response

    def status_callback(self, _request: Trigger.Request,
                        response: Trigger.Response) -> Trigger.Response:
        if self.finished:
            state = 'finished'
        elif self.enabled and self.current_goal is not None:
            state = 'exploring'
        elif self.enabled:
            state = 'idle'
        else:
            state = 'disabled'
        response.success = True
        response.message = f'{state}; frontiers_empty_cycles={self.empty_cycles}'
        return response

    # ------------------------------------------------------------------ main loop
    def tick(self) -> None:
        if not self.enabled:
            return
        if self.autostart and not self.finished:
            if time.monotonic() - self._started_at < self.start_delay:
                return

        if self.grid is None or self.map_info is None:
            self.get_logger().info(
                f'Waiting for map on {self.map_topic}',
                throttle_duration_sec=5.0)
            return

        robot = self.robot_pose()
        if robot is None:
            self.get_logger().info(
                f'Waiting for TF {self.map_frame} <- {self.robot_frame}',
                throttle_duration_sec=5.0)
            return

        if self.current_goal is not None:
            self.check_goal(robot)
            return

        if time.monotonic() < self.next_goal_at:
            return

        candidate = self.best_frontier(robot)
        if candidate is not None:
            self.empty_cycles = 0
            self.send_goal(candidate, robot)
            return

        self.empty_cycles += 1
        self.get_logger().info(
            f'No reachable frontier (cycle {self.empty_cycles}/{self.finish_patience})',
            throttle_duration_sec=3.0)
        if self.empty_cycles >= self.finish_patience:
            self.finish()

    # ------------------------------------------------------------------ pose / goals
    def robot_pose(self) -> Optional[Tuple[float, float]]:
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.robot_frame, Time())
            return (tf.transform.translation.x, tf.transform.translation.y)
        except TransformException:
            # odom 回调的坐标只在 odom 系; map != odom 时不能直接当 map 位姿.
            if (self.last_odom is not None and
                    self.last_odom.header.frame_id == self.map_frame):
                return (self.last_odom.pose.pose.position.x,
                        self.last_odom.pose.pose.position.y)
            return None

    def make_goal_pose(self, x: float, y: float, robot: Tuple[float, float]) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0
        pose.pose.orientation = quaternion_from_yaw(
            yaw_from_points(robot[0], robot[1], x, y))
        return self.transform_goal(pose)

    def transform_goal(self, pose: PoseStamped) -> PoseStamped:
        if not self.goal_frame or self.goal_frame == pose.header.frame_id:
            return pose
        if do_transform_pose is None:
            return pose
        try:
            tf = self.tf_buffer.lookup_transform(
                self.goal_frame, pose.header.frame_id, Time())
        except TransformException:
            self.get_logger().warning(
                f'Cannot transform goal {pose.header.frame_id} -> {self.goal_frame}; '
                'sending in source frame',
                throttle_duration_sec=5.0)
            return pose
        transformed = do_transform_pose(pose, tf)
        transformed.header.frame_id = self.goal_frame
        transformed.header.stamp = self.get_clock().now().to_msg()
        return transformed

    def send_goal(self, candidate: FrontierCandidate,
                  robot: Tuple[float, float]) -> None:
        pose = self.make_goal_pose(candidate.x, candidate.y, robot)
        self.current_goal = pose
        self.current_goal_xy = (candidate.x, candidate.y)
        self.goal_sent_at = time.monotonic()
        self.goal_accepted = False
        stamp = pose.header.stamp
        self.goal_expected_stamp_ns = (
            int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec))
        self.goal_seq += 1
        seq = self.goal_seq
        self.get_logger().info(
            f'Frontier goal ({candidate.x:.2f}, {candidate.y:.2f}) '
            f'size={candidate.size} dist={candidate.distance:.2f}m '
            f'score={candidate.score:.2f}')

        use_nav2 = False
        if self.goal_interface == 'nav2':
            use_nav2 = self.action_client is not None
        elif self.goal_interface == 'auto':
            use_nav2 = (self.action_client is not None
                        and self.action_client.server_is_ready())

        if use_nav2:
            goal = NavigateToPose.Goal()
            goal.pose = pose
            self.goal_mode = 'nav2'
            future = self.action_client.send_goal_async(goal)
            future.add_done_callback(lambda f, s=seq: self.goal_response_cb(f, s))
        else:
            if self.goal_interface == 'nav2':
                self.get_logger().warning(
                    'Nav2 action server unavailable; falling back to goal topic',
                    throttle_duration_sec=5.0)
            self.goal_mode = 'pose'
            self.goal_pub.publish(pose)

    def goal_response_cb(self, future, seq: int) -> None:
        if seq != self.goal_seq:
            return
        try:
            handle = future.result()
        except Exception as exc:  # noqa: BLE001 - report action failure
            self.finish_goal(False, f'goal send failed: {exc}')
            return
        if not handle.accepted:
            self.finish_goal(False, 'goal rejected')
            return
        self.goal_accepted = True
        self.nav2_goal_handle = handle
        handle.get_result_async().add_done_callback(
            lambda f, s=seq: self.goal_result_cb(f, s))

    def goal_result_cb(self, future, seq: int) -> None:
        if seq != self.goal_seq:
            return
        try:
            status = future.result().status
        except Exception as exc:  # noqa: BLE001 - report action failure
            self.finish_goal(False, f'goal result failed: {exc}')
            return
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.finish_goal(True, 'nav2 goal reached')
        else:
            self.finish_goal(False, f'nav2 goal failed (status={status})')

    def goal_status_cb(self, msg: String) -> None:
        if self.current_goal is None:
            return
        raw = msg.data.strip()
        status, token = parse_goal_status(raw)
        # 目标带时间戳 token: 旧目标的迟到状态不能作用到新目标.
        if (token is not None and self.goal_expected_stamp_ns is not None and
                token != self.goal_expected_stamp_ns):
            return
        if status == 'accepted':
            self.goal_accepted = True
        elif status == 'failed':
            self.finish_goal(False, 'navigation reported failure')
        elif status == 'reached':
            self.finish_goal(True, 'navigation reported reached')

    def check_goal(self, robot: Tuple[float, float]) -> None:
        if self.current_goal_xy is None:
            self.current_goal = None
            return
        elapsed = time.monotonic() - self.goal_sent_at
        dist = math.hypot(robot[0] - self.current_goal_xy[0],
                          robot[1] - self.current_goal_xy[1])
        if self.goal_mode == 'pose':
            if dist <= self.goal_tolerance:
                self.finish_goal(True, f'goal reached ({dist:.2f} m)')
                return
            if (not self.goal_accepted and
                    elapsed > self.goal_accept_timeout):
                self.finish_goal(
                    False,
                    f'navigation did not accept goal within '
                    f'{self.goal_accept_timeout:.1f}s')
                return
        if elapsed > self.goal_timeout:
            self._cancel_nav2_goal()
            self.finish_goal(False, f'goal timeout after {elapsed:.1f}s '
                                    f'(remaining {dist:.2f} m)')

    def finish_goal(self, success: bool, reason: str) -> None:
        # 超时/本地失败时用带 goal token 的 stop 停止旧目标. 如果新目标
        # 已经先到达导航节点, token 不匹配, 不会误停新目标.
        if not success and self.goal_mode == 'pose':
            self.request_stop_goal()
        if self.current_goal_xy is not None:
            key = (round(self.current_goal_xy[0] * 10),
                   round(self.current_goal_xy[1] * 10))
            if success:
                self.goal_failures.pop(key, None)
                blacklist_for = self.blacklist_duration
            else:
                failures = self.goal_failures.get(key, 0) + 1
                self.goal_failures[key] = failures
                blacklist_for = blacklist_duration_for(
                    failures, self.first_failure_blacklist,
                    self.blacklist_duration)
                self.get_logger().info(
                    f'Goal failure {failures} at '
                    f'({self.current_goal_xy[0]:.2f}, '
                    f'{self.current_goal_xy[1]:.2f}); '
                    f'blacklist {blacklist_for:.1f}s')
            self.blacklist.append((self.current_goal_xy[0],
                                   self.current_goal_xy[1],
                                   time.monotonic() + blacklist_for))
        self.get_logger().info(('Goal succeeded: ' if success else 'Goal failed: ') + reason)
        self.goal_seq += 1  # invalidate late action callbacks
        self.current_goal = None
        self.current_goal_xy = None
        self.goal_mode = ''
        self.goal_accepted = False
        self.goal_expected_stamp_ns = None
        self.nav2_goal_handle = None
        self.next_goal_at = time.monotonic() + self.goal_cooldown
        self.cleanup_blacklist()

    def request_stop_goal(self) -> None:
        msg = String()
        if self.goal_expected_stamp_ns is not None:
            msg.data = str(self.goal_expected_stamp_ns)
        self.stop_pub.publish(msg)

    def request_stop(self) -> None:
        if not self.stop_client.service_is_ready():
            self.get_logger().warning(
                'Stop service /stop_navigation not available',
                throttle_duration_sec=5.0)
            return
        self.stop_client.call_async(Trigger.Request())

    def _cancel_nav2_goal(self) -> None:
        if self.nav2_goal_handle is not None:
            self.nav2_goal_handle.cancel_goal_async()
            self.nav2_goal_handle = None

    def cleanup_blacklist(self) -> None:
        now = time.monotonic()
        self.blacklist = [b for b in self.blacklist if b[2] > now]

    def is_blacklisted(self, x: float, y: float) -> bool:
        for bx, by, _ in self.blacklist:
            if math.hypot(x - bx, y - by) <= self.blacklist_radius:
                return True
        return False

    # ------------------------------------------------------------------ frontier logic
    def best_frontier(self, robot: Tuple[float, float]) -> Optional[FrontierCandidate]:
        if self.grid is None or self.map_info is None:
            return None
        grid = self.grid
        info = self.map_info
        resolution = float(info.resolution)
        if resolution <= 0.0:
            return None
        origin_x = float(info.origin.position.x)
        origin_y = float(info.origin.position.y)

        free = (grid >= 0) & (grid < self.free_thresh)
        unknown = grid < 0
        if not free.any() or not unknown.any():
            if self.publish_markers:
                self.publish_frontier_markers([], None)
            return None

        frontier_mask = free & dilate_unknown(unknown)
        clusters = extract_clusters(frontier_mask, self.min_frontier_cells)
        if not clusters:
            if self.publish_markers:
                self.publish_frontier_markers([], None)
            return None

        occupied = grid >= self.occupied_thresh
        integral = build_integral(occupied)
        unknown_integral = build_integral(unknown)
        h, w = grid.shape

        robot_col = int((robot[0] - origin_x) / resolution)
        robot_row = int((robot[1] - origin_y) / resolution)
        start = nearest_free_cell(free, robot_row, robot_col)
        if start is None:
            return None
        reachable = reachable_free_mask(free, start) if self.check_reachability else free

        # 用导航节点的 /global_costmap 做第二重可达性检查. 它含 robot_radius
        # 硬边界 (紫色 100) 和 inflation_radius 蓝区 (<100 可通行), 与 A*
        # 完全一致. 只用 /map 的 free 判断会把 0.5m 门洞外的目标误判为可达.
        nav_reachable = None
        nav_grid = None
        if (self.check_reachability and self.nav_grid is not None and
                self.nav_info is not None and
                self.nav_grid.shape == grid.shape and
                abs(float(self.nav_info.resolution) - resolution) < 1e-9 and
                abs(float(self.nav_info.origin.position.x) - origin_x) < 1e-6 and
                abs(float(self.nav_info.origin.position.y) - origin_y) < 1e-6):
            nav_grid = self.nav_grid
            nav_free = (nav_grid >= 0) & (nav_grid < 100)
            nav_start = nearest_free_cell(nav_free, robot_row, robot_col)
            if nav_start is not None:
                key = (self._nav_seq, nav_start)
                if key != self._nav_reachable_key:
                    self._nav_reachable_cache = reachable_free_mask(nav_free, nav_start)
                    self._nav_reachable_key = key
                nav_reachable = self._nav_reachable_cache

        clearance_cells = max(1, int(math.ceil(self.goal_clearance / resolution)))
        backoff_cells = max(0, int(math.ceil(self.goal_backoff / resolution)))
        search_cells = backoff_cells + max(4, int(math.ceil(0.8 / resolution)))
        candidates: List[FrontierCandidate] = []

        for cells in clusters:
            mean_row = sum(r for r, _ in cells) / len(cells)
            mean_col = sum(c for _, c in cells) / len(cells)

            # 目标向自由区回退: 在 cluster 的整个包围盒(外扩搜索距离)里
            # 找距 unknown 至少 goal_backoff、且离占据格有 goal_clearance 的
            # 自由格. 不能只围绕 cluster 均值搜索, 大前沿的均值可能落在
            # 远处, 也不能退回边界前沿格(那会落进 costmap 紫色/unknown).
            if backoff_cells <= 0:
                continue
            rows = [r for r, _ in cells]
            cols = [c for _, c in cells]
            r_lo = max(0, min(rows) - search_cells)
            r_hi = min(h, max(rows) + search_cells + 1)
            c_lo = max(0, min(cols) - search_cells)
            c_hi = min(w, max(cols) + search_cells + 1)
            best_cell = None
            best_d2 = float('inf')
            for row in range(r_lo, r_hi):
                for col in range(c_lo, c_hi):
                    if not free[row, col] or not reachable[row, col]:
                        continue
                    if (nav_reachable is not None and
                            (not nav_reachable[row, col] or nav_grid[row, col] >= 100)):
                        continue
                    r0 = max(0, row - clearance_cells)
                    c0 = max(0, col - clearance_cells)
                    r1 = min(h, row + clearance_cells + 1)
                    c1 = min(w, col + clearance_cells + 1)
                    if window_sum(integral, r0, c0, r1, c1) != 0:
                        continue
                    u0 = max(0, row - backoff_cells)
                    v0 = max(0, col - backoff_cells)
                    u1 = min(h, row + backoff_cells + 1)
                    v1 = min(w, col + backoff_cells + 1)
                    if window_sum(unknown_integral, u0, v0, u1, v1) != 0:
                        continue
                    d2 = (row - mean_row) ** 2 + (col - mean_col) ** 2
                    if d2 < best_d2:
                        best_d2 = d2
                        best_cell = (row, col)
            if best_cell is None:
                # 该 cluster 附近没有满足 backoff 的自由格, 跳过; 不要退回
                # 贴着 unknown 的前沿格, 否则导航会报 goal in obstacle.
                continue
            row, col = best_cell
            x, y = cell_to_world(row, col, origin_x, origin_y, resolution)
            dist = math.hypot(x - robot[0], y - robot[1])
            if dist < self.min_goal_distance or dist > self.max_goal_distance:
                continue
            if self.is_blacklisted(x, y):
                continue
            size = len(cells)
            nav_cost = int(nav_grid[row, col]) if nav_grid is not None else 0
            score = (dist - self.frontier_size_weight * min(size, 500) +
                     self.nav_cost_weight * nav_cost / 100.0)
            candidates.append(FrontierCandidate(
                x=x, y=y, size=size, distance=dist, score=score,
                row=row, col=col))

        if not candidates:
            if self.publish_markers:
                self.publish_frontier_markers([], None)
            return None

        best = min(candidates, key=lambda c: c.score)
        if self.publish_markers:
            self.publish_frontier_markers(candidates, best)
        return best

    def publish_frontier_markers(self, candidates: Sequence[FrontierCandidate],
                                 best: Optional[FrontierCandidate]) -> None:
        markers = MarkerArray()
        delete = Marker()
        delete.header.frame_id = self.map_frame
        delete.header.stamp = self.get_clock().now().to_msg()
        delete.ns = 'frontier_candidates'
        delete.action = Marker.DELETEALL
        markers.markers.append(delete)

        for index, candidate in enumerate(candidates):
            marker = Marker()
            marker.header.frame_id = self.map_frame
            marker.header.stamp = delete.header.stamp
            marker.ns = 'frontier_candidates'
            marker.id = index
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = candidate.x
            marker.pose.position.y = candidate.y
            marker.pose.position.z = 0.15
            marker.pose.orientation.w = 1.0
            scale = 0.18 if best is None or candidate is not best else 0.30
            marker.scale.x = marker.scale.y = marker.scale.z = scale
            if best is not None and candidate is best:
                marker.color.r, marker.color.g, marker.color.b = 1.0, 0.1, 0.1
            else:
                marker.color.r, marker.color.g, marker.color.b = 0.1, 0.9, 0.2
            marker.color.a = 0.9
            markers.markers.append(marker)

        self.marker_pub.publish(markers)

    # ------------------------------------------------------------------ finish
    def finish(self) -> None:
        if self.finished:
            return
        self.finished = True
        self.enabled = False
        self.request_stop()
        self.get_logger().info('Exploration finished: no reachable frontiers remain')
        if self.save_map_on_finish:
            self.save_map()

    def save_map(self) -> None:
        path = os.path.abspath(os.path.expanduser(self.save_map_path))
        directory = os.path.dirname(path)
        if directory:
            os.makedirs(directory, exist_ok=True)
        cmd = ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', path]
        if bool(self.get_parameter('use_sim_time').value):
            cmd += ['--ros-args', '-p', 'use_sim_time:=true']
        self.get_logger().info('Saving map with: ' + ' '.join(cmd))
        try:
            subprocess.Popen(cmd, stdout=subprocess.DEVNULL,
                             stderr=subprocess.STDOUT)
        except OSError as exc:
            self.get_logger().error(f'Failed to start map_saver_cli: {exc}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
