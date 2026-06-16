"""System Manager - Tracks subsystem health and lifecycle.

Monitors the status of simulator, SLAM, and navigation subsystems
by checking topic activity and node availability.
"""

import threading
from enum import Enum
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class SubsystemStatus(Enum):
    """Status of a subsystem."""
    UNKNOWN = "unknown"
    ONLINE = "online"
    OFFLINE = "offline"
    ERROR = "error"


class SubsystemInfo:
    """Information about a tracked subsystem."""

    def __init__(
        self,
        name: str,
        topic: str,
        msg_type: type,
        timeout: float = 5.0,
    ) -> None:
        """Initialize subsystem info.

        Args:
            name: Human-readable subsystem name.
            topic: ROS2 topic to monitor for activity.
            msg_type: Message type of the monitored topic.
            timeout: Seconds without a message before marking offline.
        """
        self.name: str = name
        self.topic: str = topic
        self.msg_type: type = msg_type
        self.timeout: float = timeout
        self.status: SubsystemStatus = SubsystemStatus.UNKNOWN
        self.last_msg_time: float = 0.0
        self.msg_count: int = 0
        self._lock: threading.Lock = threading.Lock()

    def update_activity(self) -> None:
        """Record a message received on the monitored topic."""
        import time
        with self._lock:
            self.last_msg_time = time.time()
            self.msg_count += 1
            self.status = SubsystemStatus.ONLINE

    def check_timeout(self) -> None:
        """Check if the subsystem has timed out and update status."""
        import time
        with self._lock:
            if self.status == SubsystemStatus.ONLINE:
                elapsed = time.time() - self.last_msg_time
                if elapsed > self.timeout:
                    self.status = SubsystemStatus.OFFLINE


class SystemManager:
    """Manages and monitors subsystem health.

    Tracks the online/offline status of simulator, SLAM, and navigation
    by subscribing to key topics and monitoring message activity.
    """

    def __init__(self, node: Node) -> None:
        """Initialize the SystemManager.

        Args:
            node: The parent ROS2 node to create subscribers on.
        """
        self._node = node
        self._logger = node.get_logger()
        self._subsystems: Dict[str, SubsystemInfo] = {}
        self._subscribers: Dict[str, object] = {}

    def register_subsystem(
        self,
        name: str,
        topic: str,
        msg_type: type,
        timeout: float = 5.0,
    ) -> None:
        """Register a subsystem for monitoring.

        Args:
            name: Identifier for the subsystem (e.g. 'simulator').
            topic: Topic to monitor for activity.
            msg_type: Message type of the monitored topic.
            timeout: Seconds without messages before marking offline.
        """
        info = SubsystemInfo(name, topic, msg_type, timeout)
        self._subsystems[name] = info

        # Create subscriber to monitor topic activity
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        sub = self._node.create_subscription(
            msg_type,
            topic,
            lambda msg, n=name: self._on_subsystem_msg(n),
            qos,
        )
        self._subscribers[name] = sub
        self._logger.info(
            f"Registered subsystem '{name}' monitoring topic '{topic}'"
        )

    def _on_subsystem_msg(self, name: str) -> None:
        """Callback when a subsystem topic receives a message.

        Args:
            name: The subsystem identifier.
        """
        if name in self._subsystems:
            self._subsystems[name].update_activity()

    def check_health(self) -> Dict[str, str]:
        """Check health of all registered subsystems.

        Returns:
            Dictionary mapping subsystem name to status string.
        """
        result: Dict[str, str] = {}
        for name, info in self._subsystems.items():
            info.check_timeout()
            result[name] = info.status.value
        return result

    def get_subsystem_status(self, name: str) -> SubsystemStatus:
        """Get the status of a specific subsystem.

        Args:
            name: The subsystem identifier.

        Returns:
            Current SubsystemStatus of the subsystem.
        """
        if name not in self._subsystems:
            return SubsystemStatus.UNKNOWN
        self._subsystems[name].check_timeout()
        return self._subsystems[name].status

    def is_subsystem_online(self, name: str) -> bool:
        """Check if a specific subsystem is online.

        Args:
            name: The subsystem identifier.

        Returns:
            True if the subsystem status is ONLINE.
        """
        return self.get_subsystem_status(name) == SubsystemStatus.ONLINE

    def get_all_online(self) -> bool:
        """Check if all registered subsystems are online.

        Returns:
            True if every subsystem has ONLINE status.
        """
        health = self.check_health()
        return all(s == SubsystemStatus.ONLINE.value for s in health.values())

    def get_summary(self) -> Dict[str, object]:
        """Get a full summary of all subsystems.

        Returns:
            Dictionary with subsystem names as keys and status details
            including message counts.
        """
        health = self.check_health()
        summary: Dict[str, object] = {}
        for name, info in self._subsystems.items():
            summary[name] = {
                "status": info.status.value,
                "topic": info.topic,
                "msg_count": info.msg_count,
            }
        return summary

    def wait_for_subsystem(
        self,
        name: str,
        timeout: float = 30.0,
        poll_interval: float = 0.5,
    ) -> bool:
        """Wait for a subsystem to come online.

        Args:
            name: The subsystem identifier.
            timeout: Maximum seconds to wait.
            poll_interval: Seconds between checks.

        Returns:
            True if the subsystem came online within the timeout.
        """
        import time
        start = time.time()
        while (time.time() - start) < timeout:
            if self.is_subsystem_online(name):
                self._logger.info(f"Subsystem '{name}' is online.")
                return True
            time.sleep(poll_interval)
        self._logger.warn(
            f"Subsystem '{name}' did not come online within {timeout}s."
        )
        return False
