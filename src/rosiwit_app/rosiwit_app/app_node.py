#!/usr/bin/env python3
"""
RosiwitAppNode - 移动机器人统一调度主节点
状态机管理整个系统：INIT → INITIALIZING → READY → NAVIGATING → READY
                                                ↕ MAPPING
"""

import json
import math
import os
from enum import Enum
from typing import Optional

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import String
from std_srvs.srv import Trigger
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Odometry

from rosiwit_app.map_manager import MapManager
from rosiwit_app.navigation_manager import NavigationManager
from rosiwit_app.waypoint_server import WaypointServer


class SystemState(Enum):
    """系统状态枚举"""
    INIT = "INIT"
    INITIALIZING = "INITIALIZING"
    MAPPING = "MAPPING"
    MAP_SAVING = "MAP_SAVING"
    READY = "READY"
    NAVIGATING = "NAVIGATING"
    ERROR = "ERROR"


class RosiwitAppNode(Node):
    """移动机器人统一调度主节点"""

    def __init__(self):
        super().__init__('rosiwit_app_node')

        # 声明参数
        self.declare_parameter('map_path', '/tmp/rosiwit_sim_map')
        self.declare_parameter('map_file', 'fast_lio2_map')
        self.declare_parameter('auto_save_position', True)
        self.declare_parameter('position_save_interval', 5.0)
        self.declare_parameter('startup_timeout', 30.0)
        self.declare_parameter('health_check_interval', 1.0)
        self.declare_parameter('navigation_timeout', 120.0)
        self.declare_parameter('auto_load_map', True)
        self.declare_parameter('auto_start_mapping', True)
        self.declare_parameter('waypoints_file', '')

        # 回调组（允许并发）
        self._callback_group = ReentrantCallbackGroup()

        # 状态
        self._state = SystemState.INIT
        self._current_position = {"x": 0.0, "y": 0.0, "z": 0.0,
                                  "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0}
        self._subsystem_status = {
            "simulator": False,
            "slam": False,
            "navigation": False
        }

        # 获取参数
        map_path = self.get_parameter('map_path').value
        map_file = self.get_parameter('map_file').value
        waypoints_file = self.get_parameter('waypoints_file').value

        # 初始化子模块
        self._map_manager = MapManager(map_path, map_file)
        self._navigation_manager = NavigationManager(self)

        # 航点服务器 - 优先从 share 目录加载
        if not waypoints_file:
            # 尝试从安装目录加载
            try:
                from ament_index_python.packages import get_package_share_directory
                share_dir = get_package_share_directory('rosiwit_app')
                waypoints_file = os.path.join(share_dir, 'config', 'waypoints.yaml')
            except Exception:
                waypoints_file = os.path.join(
                    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                    'config', 'waypoints.yaml')
        self._waypoint_server = WaypointServer(waypoints_file)
        self.get_logger().info(
            f"Loaded {len(self._waypoint_server.list_waypoint_names())} waypoints")

        # ====== 创建 ROS2 接口 ======

        # 订阅 - 里程计
        self._odom_sub = self.create_subscription(
            Odometry, '/odom_estimated',
            self._odom_callback, 10)

        # 订阅 - 导航目标（直接发送PoseStamped）
        self._goal_sub = self.create_subscription(
            PoseStamped, '/rosiwit_app/goal_pose',
            self._goal_pose_callback, 10)

        # 订阅 - 航点名称导航
        self._waypoint_goal_sub = self.create_subscription(
            String, '/rosiwit_app/go_to',
            self._go_to_callback, 10)

        # 发布 - 系统状态
        self._status_pub = self.create_publisher(String, '/rosiwit_app/status', 10)

        # 发布 - 初始位姿（给SLAM用）
        self._initial_pose_pub = self.create_publisher(Pose, '/initial_pose', 10)

        # 服务 - 保存地图
        self._save_map_srv = self.create_service(
            Trigger, '/rosiwit_app/save_map',
            self._save_map_callback,
            callback_group=self._callback_group)

        # 服务 - 获取状态
        self._get_status_srv = self.create_service(
            Trigger, '/rosiwit_app/get_status',
            self._get_status_callback,
            callback_group=self._callback_group)

        # 服务 - 列出航点
        self._list_waypoints_srv = self.create_service(
            Trigger, '/rosiwit_app/list_waypoints',
            self._list_waypoints_callback,
            callback_group=self._callback_group)

        # Timer - 状态监控和发布
        health_interval = self.get_parameter('health_check_interval').value
        self._status_timer = self.create_timer(health_interval, self._status_timer_callback)

        # Timer - 位置保存
        if self.get_parameter('auto_save_position').value:
            pos_interval = self.get_parameter('position_save_interval').value
            self._position_save_timer = self.create_timer(
                pos_interval, self._position_save_callback)

        # 初始化 NavigationManager
        self._navigation_manager.initialize()

        # 启动初始化（Humble没有one_shot，手动取消）
        self._startup_timer = self.create_timer(2.0, self._startup_callback)

        self.get_logger().info("=" * 50)
        self.get_logger().info("RosiwitAppNode initialized")
        self.get_logger().info(f"  Map path: {map_path}")
        self.get_logger().info(f"  Map file: {map_file}")
        self.get_logger().info(f"  Waypoints file: {waypoints_file}")
        self.get_logger().info("=" * 50)

    # ==================== 状态机 ====================

    def _set_state(self, new_state: SystemState):
        """切换状态"""
        old_state = self._state
        self._state = new_state
        self.get_logger().info(f"State: {old_state.value} → {new_state.value}")

    def _startup_callback(self):
        """启动初始化流程"""
        self._startup_timer.cancel()  # one-shot
        self.get_logger().info("Starting system initialization...")
        self._set_state(SystemState.INITIALIZING)

        # 检查是否有保存的地图
        if self._map_manager.has_saved_map():
            self.get_logger().info("Found saved map, loading...")
            self._initialize_with_map()
        else:
            self.get_logger().info("No saved map found.")
            if self.get_parameter('auto_start_mapping').value:
                self.get_logger().info("Starting mapping mode (SLAM auto-builds map)")
                self._set_state(SystemState.MAPPING)
                self.get_logger().info(
                    "SLAM is building map. Call /rosiwit_app/save_map when done.")
            else:
                self.get_logger().warn("Auto-mapping disabled. Waiting for manual map load.")
                self._set_state(SystemState.READY)

    def _initialize_with_map(self):
        """使用已保存地图初始化"""
        # 1. 加载上次位置
        position = self._map_manager.load_position()
        if position:
            self.get_logger().info(
                f"Loaded last position: x={position['x']:.2f}, y={position['y']:.2f}")
            self._current_position = position
        else:
            self.get_logger().info("No saved position, using origin (0, 0)")
            position = {"x": 0.0, "y": 0.0, "z": 0.0,
                        "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0}

        # 2. 发布初始位姿给 SLAM（延迟发布，等SLAM节点启动）
        def _pub_pose_and_cancel():
            self._publish_initial_pose(position)
            self._init_pose_timer.cancel()

        self._init_pose_timer = self.create_timer(3.0, _pub_pose_and_cancel)

        # 3. 等待导航系统就绪
        def _check_nav_and_cancel():
            self._check_navigation_ready()
            self._nav_ready_timer.cancel()

        self._nav_ready_timer = self.create_timer(5.0, _check_nav_and_cancel)

        self.get_logger().info("Map loaded. System will be READY after initialization.")

    def _publish_initial_pose(self, position: dict):
        """发布初始位姿给SLAM"""
        pose = Pose()
        pose.position.x = position.get("x", 0.0)
        pose.position.y = position.get("y", 0.0)
        pose.position.z = position.get("z", 0.0)
        pose.orientation.x = position.get("qx", 0.0)
        pose.orientation.y = position.get("qy", 0.0)
        pose.orientation.z = position.get("qz", 0.0)
        pose.orientation.w = position.get("qw", 1.0)

        self._initial_pose_pub.publish(pose)
        self.get_logger().info(
            f"Published initial pose: ({pose.position.x:.2f}, {pose.position.y:.2f})")

    def _check_navigation_ready(self):
        """检查导航系统就绪状态"""
        if self._navigation_manager.wait_for_server(timeout_sec=5.0):
            self.get_logger().info("Navigation system is ready")
            self._subsystem_status["navigation"] = True
        else:
            self.get_logger().warn("Navigation system not yet available")

        if self._state == SystemState.INITIALIZING:
            self._set_state(SystemState.READY)

    # ==================== 回调函数 ====================

    def _odom_callback(self, msg: Odometry):
        """里程计回调 - 更新当前位置"""
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        self._current_position = {
            "x": pos.x, "y": pos.y, "z": pos.z,
            "qx": ori.x, "qy": ori.y, "qz": ori.z, "qw": ori.w
        }
        self._subsystem_status["slam"] = True

    def _goal_pose_callback(self, msg: PoseStamped):
        """接收导航目标（PoseStamped）"""
        self.get_logger().info(
            f"Received navigation goal: "
            f"x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}, "
            f"frame={msg.header.frame_id}")
        self._start_navigation(msg)

    def _go_to_callback(self, msg: String):
        """接收航点名称，导航到对应位置"""
        waypoint_name = msg.data.strip()
        self.get_logger().info(f"Received go_to request: '{waypoint_name}'")

        pose = self._waypoint_server.to_pose_stamped(waypoint_name)
        if pose is None:
            available = ", ".join(self._waypoint_server.list_waypoint_names())
            self.get_logger().warn(
                f"Unknown waypoint '{waypoint_name}'. Available: {available}")
            return

        self.get_logger().info(f"Navigating to waypoint '{waypoint_name}'")
        self._start_navigation(pose)

    def _start_navigation(self, pose_stamped: PoseStamped):
        """开始导航"""
        if self._state not in (SystemState.READY, SystemState.NAVIGATING):
            self.get_logger().warn(
                f"Cannot start navigation in state {self._state.value}")
            return

        # 确保 frame_id 设置
        if not pose_stamped.header.frame_id:
            pose_stamped.header.frame_id = "map"

        self._set_state(SystemState.NAVIGATING)
        success = self._navigation_manager.send_goal(
            pose_stamped, on_goal_reached=self._on_navigation_done)

        if not success:
            self.get_logger().error("Failed to send navigation goal")
            self._set_state(SystemState.READY)

    def _on_navigation_done(self, success: bool):
        """导航完成回调"""
        if success:
            self.get_logger().info("Navigation completed successfully!")
        else:
            self.get_logger().warn("Navigation failed or was cancelled")

        self._set_state(SystemState.READY)

        # 保存到达后的位置
        pos = self._current_position
        self._map_manager.save_position(
            pos["x"], pos["y"], pos["z"],
            pos["qx"], pos["qy"], pos["qz"], pos["qw"])

    # ==================== 服务回调 ====================

    def _save_map_callback(self, request, response):
        """保存地图服务回调"""
        self.get_logger().info("Save map requested...")

        if self._state not in (SystemState.MAPPING, SystemState.READY):
            response.success = False
            response.message = f"Cannot save map in state {self._state.value}"
            return response

        # 调用 SLAM 的 /save_map 服务
        try:
            from rclpy.client import Client
            slam_save_client = self.create_client(Trigger, '/save_map')
            if not slam_save_client.wait_for_service(timeout_sec=5.0):
                response.success = False
                response.message = "SLAM /save_map service not available"
                return response

            future = slam_save_client.call_async(Trigger.Request())
            import time
            start = time.time()
            while not future.done():
                if time.time() - start > 15.0:
                    response.success = False
                    response.message = "Save map timeout"
                    return response
                time.sleep(0.1)

            result = future.result()
            if result and result.success:
                # 同时保存当前位置
                pos = self._current_position
                self._map_manager.save_position(
                    pos["x"], pos["y"], pos["z"],
                    pos["qx"], pos["qy"], pos["qz"], pos["qw"])

                response.success = True
                response.message = f"Map saved to {self._map_manager.map_path}"
                self.get_logger().info("Map saved successfully")

                # 如果之前在MAPPING状态，切换到READY
                if self._state == SystemState.MAPPING:
                    self._set_state(SystemState.READY)
                    # 重新检查导航就绪
                    self._check_navigation_ready()
            else:
                response.success = False
                response.message = result.message if result else "Unknown error"
        except Exception as e:
            response.success = False
            response.message = f"Error: {str(e)}"

        return response

    def _get_status_callback(self, request, response):
        """获取系统状态"""
        status = self._build_status_dict()
        response.success = True
        response.message = json.dumps(status, indent=2)
        return response

    def _list_waypoints_callback(self, request, response):
        """列出所有航点"""
        waypoints = self._waypoint_server.list_waypoints()
        result = {}
        for name, wp in waypoints.items():
            result[name] = {
                "x": wp["x"], "y": wp["y"],
                "description": wp.get("description", "")
            }
        response.success = True
        response.message = json.dumps(result, indent=2)
        return response

    # ==================== 定时器回调 ====================

    def _status_timer_callback(self):
        """状态监控定时器 - 发布状态并检查子系统"""
        # 检查子系统状态
        self._check_subsystems()

        # 发布状态
        status = self._build_status_dict()
        msg = String()
        msg.data = json.dumps(status)
        self._status_pub.publish(msg)

    def _position_save_callback(self):
        """位置保存定时器"""
        if self._current_position:
            pos = self._current_position
            self._map_manager.save_position(
                pos["x"], pos["y"], pos["z"],
                pos["qx"], pos["qy"], pos["qz"], pos["qw"])

    # ==================== 辅助方法 ====================

    def _check_subsystems(self):
        """检查各子系统是否在线"""
        # 检查 SLAM - 通过是否有里程计数据
        # (odom_callback 中会更新)

        # 检查 Navigation - 通过 action server
        if self._navigation_manager.is_server_online():
            self._subsystem_status["navigation"] = True

    def _build_status_dict(self) -> dict:
        """构建状态字典"""
        pos = self._current_position
        yaw = MapManager._quaternion_to_yaw(
            pos["qx"], pos["qy"], pos["qz"], pos["qw"])

        nav_feedback = self._navigation_manager.get_feedback()

        return {
            "state": self._state.value,
            "position": {
                "x": round(pos["x"], 3),
                "y": round(pos["y"], 3),
                "yaw": round(yaw, 3)
            },
            "map_loaded": self._map_manager.has_saved_map(),
            "navigation_active": self._navigation_manager.is_navigating(),
            "navigation_feedback": nav_feedback,
            "waypoint_count": len(self._waypoint_server.list_waypoint_names()),
            "waypoints": self._waypoint_server.list_waypoint_names(),
            "subsystems": dict(self._subsystem_status)
        }


def main(args=None):
    import rclpy
    from rclpy.executors import ExternalShutdownException
    rclpy.init(args=args)
    node = RosiwitAppNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
