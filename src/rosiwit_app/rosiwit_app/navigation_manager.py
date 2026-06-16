"""
NavigationManager - 导航管理器
封装 Nav2 NavigateToPose Action Client，管理导航目标和状态跟踪
支持 Nav2 未安装时的优雅降级
"""

import threading
from typing import Optional, Callable

from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

# 尝试导入 Nav2 消息（可能未安装）
try:
    from nav2_msgs.action import NavigateToPose
    from rclpy.action import ActionClient
    NAV2_AVAILABLE = True
except ImportError:
    NAV2_AVAILABLE = False


class NavigationManager:
    """管理 Nav2 导航 Action Client，支持优雅降级"""

    def __init__(self, node: Node):
        self._node = node
        self._logger = node.get_logger()

        if not NAV2_AVAILABLE:
            self._logger.warn(
                "NavigationManager: nav2_msgs not available. "
                "Navigation features will be disabled. "
                "Install ros-humble-nav2-msgs to enable.")
            self._action_client = None
            self._send_goal_future = None
            self._get_result_future = None
            self._goal_handle = None
            self._is_navigating = False
            self._current_goal = None
            self._navigation_result = None
            self._distance_remaining = 0.0
            self._navigation_feedback = None
            self._on_goal_reached = None
            return

        # Action Client
        self._action_client = None
        self._send_goal_future = None
        self._get_result_future = None
        self._goal_handle = None

        # 状态
        self._is_navigating = False
        self._current_goal: Optional[PoseStamped] = None
        self._navigation_result: Optional[bool] = None
        self._distance_remaining: float = 0.0
        self._navigation_feedback = None

        # 回调
        self._on_goal_reached: Optional[Callable[[bool], None]] = None

    def initialize(self) -> bool:
        """初始化 Action Client"""
        if not NAV2_AVAILABLE:
            return False
        try:
            self._action_client = ActionClient(
                self._node, NavigateToPose, 'navigate_to_pose')
            self._logger.info("NavigationManager: ActionClient created for /navigate_to_pose")
            return True
        except Exception as e:
            self._logger.error(f"NavigationManager: Failed to create ActionClient: {e}")
            return False

    def wait_for_server(self, timeout_sec: float = 10.0) -> bool:
        """等待 Nav2 Action Server 就绪"""
        if not NAV2_AVAILABLE or self._action_client is None:
            return False
        self._logger.info(f"NavigationManager: Waiting for NavigateToPose server (timeout={timeout_sec}s)...")
        ready = self._action_client.wait_for_server(timeout_sec=timeout_sec)
        if ready:
            self._logger.info("NavigationManager: NavigateToPose server is ready")
        else:
            self._logger.warn("NavigationManager: NavigateToPose server NOT available")
        return ready

    def send_goal(self, pose_stamped: PoseStamped,
                  on_goal_reached: Optional[Callable[[bool], None]] = None) -> bool:
        """
        发送导航目标
        Args:
            pose_stamped: 目标位姿
            on_goal_reached: 导航完成回调 (success: bool)
        Returns:
            是否成功发送
        """
        if not NAV2_AVAILABLE:
            self._logger.error("NavigationManager: nav2_msgs not available, cannot navigate")
            return False

        if self._action_client is None:
            self._logger.error("NavigationManager: ActionClient not initialized")
            return False

        if not self._action_client.server_is_ready():
            if not self.wait_for_server(timeout_sec=5.0):
                self._logger.error("NavigationManager: Nav2 server not ready")
                return False

        # 如果正在导航，先取消
        if self._is_navigating:
            self.cancel_goal()

        # 构造 Goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose_stamped

        self._current_goal = pose_stamped
        self._on_goal_reached = on_goal_reached
        self._navigation_result = None
        self._distance_remaining = 0.0

        self._logger.info(
            f"NavigationManager: Sending goal -> "
            f"x={pose_stamped.pose.position.x:.2f}, "
            f"y={pose_stamped.pose.position.y:.2f}, "
            f"frame={pose_stamped.header.frame_id}")

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback)
        self._send_goal_future.add_done_callback(self._goal_response_callback)

        self._is_navigating = True
        return True

    def cancel_goal(self) -> bool:
        """取消当前导航"""
        if not self._is_navigating or self._goal_handle is None:
            return False

        self._logger.info("NavigationManager: Cancelling current goal...")
        try:
            future = self._goal_handle.cancel_goal_async()
            self._is_navigating = False
            self._goal_handle = None
            return True
        except Exception as e:
            self._logger.error(f"NavigationManager: Cancel failed: {e}")
            return False

    def is_navigating(self) -> bool:
        """是否正在导航"""
        return self._is_navigating

    def get_feedback(self) -> dict:
        """
        获取导航反馈
        Returns:
            {distance_remaining, navigating, current_goal}
        """
        return {
            "distance_remaining": self._distance_remaining,
            "navigating": self._is_navigating,
            "current_goal": {
                "x": self._current_goal.pose.position.x if self._current_goal else 0.0,
                "y": self._current_goal.pose.position.y if self._current_goal else 0.0,
            } if self._current_goal else None
        }

    def _goal_response_callback(self, future):
        """目标接受/拒绝回调"""
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self._logger.warn("NavigationManager: Goal REJECTED by server")
                self._is_navigating = False
                if self._on_goal_reached:
                    self._on_goal_reached(False)
                return

            self._goal_handle = goal_handle
            self._logger.info("NavigationManager: Goal ACCEPTED by server")
            self._get_result_future = goal_handle.get_result_async()
            self._get_result_future.add_done_callback(self._result_callback)
        except Exception as e:
            self._logger.error(f"NavigationManager: Goal response error: {e}")
            self._is_navigating = False

    def _result_callback(self, future):
        """导航结果回调"""
        try:
            result = future.result()
            status = result.status
            success = (status == 4)  # SUCCEEDED
            self._navigation_result = success

            status_names = {0: "UNKNOWN", 1: "ACCEPTED", 2: "EXECUTING",
                          3: "CANCELING", 4: "SUCCEEDED", 5: "ABORTED", 6: "CANCELED"}

            if success:
                self._logger.info("NavigationManager: Goal REACHED successfully!")
            else:
                self._logger.warn(
                    f"NavigationManager: Goal FAILED with status: {status_names.get(status, status)}")

            self._is_navigating = False
            self._goal_handle = None

            if self._on_goal_reached:
                self._on_goal_reached(success)
                self._on_goal_reached = None

        except Exception as e:
            self._logger.error(f"NavigationManager: Result callback error: {e}")
            self._is_navigating = False

    def _feedback_callback(self, feedback_msg):
        """导航反馈回调"""
        feedback = feedback_msg.feedback
        self._navigation_feedback = feedback
        self._distance_remaining = feedback.distance_remaining

    def is_server_online(self) -> bool:
        """检查 Nav2 Action Server 是否在线"""
        if not NAV2_AVAILABLE or self._action_client is None:
            return False
        return self._action_client.server_is_ready()

    @staticmethod
    def is_nav2_available() -> bool:
        """检查 Nav2 是否可用"""
        return NAV2_AVAILABLE
