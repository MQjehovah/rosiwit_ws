"""Perception module base class: heartbeat + lightweight observability.

所有感知节点（AVM / 地面纹理 / 二维码）继承本类，统一向
``/perception/heartbeat`` 发布自身状态，由 ``perception_manager`` 汇总成
``/perception/status`` 和 ``/diagnostics``。这样新增模块只需要：

    class MyModule(PerceptionModule):
        def __init__(self):
            super().__init__('my_module')
        def on_frame(self):
            self.note_frame(...)
"""

import json
import time

from rclpy.node import Node
from std_msgs.msg import String


class PerceptionModule(Node):
    def __init__(self, node_name: str, module_name: str = None):
        super().__init__(node_name)
        self.module_name = module_name or node_name

        self.declare_parameter('heartbeat_topic', '/perception/heartbeat')
        self.declare_parameter('heartbeat_rate', 1.0)
        self.declare_parameter('heartbeat_enabled', True)

        self._heartbeat_enabled = bool(
            self.get_parameter('heartbeat_enabled').value)
        rate = float(self.get_parameter('heartbeat_rate').value)
        self._heartbeat_period = 1.0 / max(rate, 0.1)
        self._heartbeat_topic = self.get_parameter(
            'heartbeat_topic').value

        self._start_time = time.monotonic()
        self._last_heartbeat = 0.0
        self._frames_at_last_hb = 0
        self._frames = 0
        self._dropped = 0
        self._last_frame_time = None
        self._last_drop_reason = ''
        self._extra_stats = {}

        self._heartbeat_pub = self.create_publisher(
            String, self._heartbeat_topic, 10)
        if self._heartbeat_enabled:
            self.create_timer(self._heartbeat_period, self._publish_heartbeat)

        self.get_logger().info(
            f'module={self.module_name} heartbeat -> {self._heartbeat_topic}')

    # ------------------------------------------------------------------ #
    # 供子类调用
    # ------------------------------------------------------------------ #
    def note_frame(self, **stats):
        """成功处理/发布一帧。"""
        self._frames += 1
        self._last_frame_time = time.monotonic()
        if stats:
            self._extra_stats.update(stats)

    def note_drop(self, reason: str = ''):
        """丢弃一帧或一次失败（解码失败、超时、无输入等）。"""
        self._dropped += 1
        self._last_drop_reason = reason

    def module_status(self) -> dict:
        now = time.monotonic()
        last_age = (None if self._last_frame_time is None
                    else round(now - self._last_frame_time, 3))
        fps = 0.0
        elapsed = now - self._last_heartbeat
        if elapsed > 0.0:
            fps = (self._frames - self._frames_at_last_hb) / elapsed
        status = {
            'module': self.module_name,
            'uptime': round(now - self._start_time, 1),
            'frames': self._frames,
            'dropped': self._dropped,
            'fps': round(fps, 2),
            'last_frame_age': last_age,
            'last_drop_reason': self._last_drop_reason,
        }
        status.update(self._extra_stats)
        return status

    # ------------------------------------------------------------------ #
    # 内部
    # ------------------------------------------------------------------ #
    def _publish_heartbeat(self):
        now = time.monotonic()
        status = self.module_status()
        status['stamp'] = round(now, 3)
        self._heartbeat_pub.publish(
            String(data=json.dumps(status, ensure_ascii=False)))
        self._last_heartbeat = now
        self._frames_at_last_hb = self._frames
