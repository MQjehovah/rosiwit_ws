"""Unit tests for SystemManager.

Tests subsystem registration, health checking, timeout detection,
and the wait_for_subsystem blocking call.
"""

import time
import threading
import unittest
from unittest.mock import MagicMock

from rosiwit_app.system_manager import SystemManager, SubsystemInfo, SubsystemStatus


class _FakeNode:
    """Minimal fake node for unit testing without full ROS2 init."""

    class _FakeLogger:
        def info(self, msg):
            pass
        def warn(self, msg):
            pass
        def error(self, msg):
            pass
        def debug(self, msg):
            pass

    def get_logger(self):
        return self._FakeLogger()

    def create_subscription(self, msg_type, topic, callback, qos_profile=10):
        """Mock create_subscription - does nothing but allow registration."""
        return MagicMock()


class TestSubsystemInfo(unittest.TestCase):
    """Test SubsystemInfo data class."""

    def test_initial_status_is_unknown(self):
        """New SubsystemInfo has UNKNOWN status."""
        info = SubsystemInfo("simulator", "/velodyne_points", object)
        self.assertEqual(info.status, SubsystemStatus.UNKNOWN)

    def test_update_activity_sets_online(self):
        """update_activity transitions status to ONLINE."""
        info = SubsystemInfo("simulator", "/velodyne_points", object)
        info.update_activity()
        self.assertEqual(info.status, SubsystemStatus.ONLINE)

    def test_update_activity_increments_count(self):
        """update_activity increments message count."""
        info = SubsystemInfo("simulator", "/velodyne_points", object)
        self.assertEqual(info.msg_count, 0)
        info.update_activity()
        self.assertEqual(info.msg_count, 1)
        info.update_activity()
        self.assertEqual(info.msg_count, 2)

    def test_check_timeout_transitions_to_offline(self):
        """check_timeout sets OFFLINE after timeout elapsed."""
        info = SubsystemInfo("simulator", "/velodyne_points", object, timeout=0.01)
        info.update_activity()
        self.assertEqual(info.status, SubsystemStatus.ONLINE)
        time.sleep(0.02)
        info.check_timeout()
        self.assertEqual(info.status, SubsystemStatus.OFFLINE)

    def test_check_timeout_stays_online_if_within_timeout(self):
        """check_timeout keeps ONLINE if within timeout period."""
        info = SubsystemInfo("simulator", "/velodyne_points", object, timeout=10.0)
        info.update_activity()
        info.check_timeout()
        self.assertEqual(info.status, SubsystemStatus.ONLINE)

    def test_check_timeout_does_not_change_unknown(self):
        """check_timeout doesn't change UNKNOWN status."""
        info = SubsystemInfo("simulator", "/velodyne_points", object, timeout=0.001)
        time.sleep(0.002)
        info.check_timeout()
        self.assertEqual(info.status, SubsystemStatus.UNKNOWN)


class TestSystemManager(unittest.TestCase):
    """Test SystemManager subsystem tracking."""

    def setUp(self):
        self.fake_node = _FakeNode()
        self.manager = SystemManager(self.fake_node)

    def test_register_subsystem(self):
        """register_subsystem adds a subsystem to tracking."""
        self.manager.register_subsystem(
            "simulator", "/velodyne_points", object, timeout=5.0
        )
        # Verify by checking status is not None
        status = self.manager.get_subsystem_status("simulator")
        self.assertEqual(status, SubsystemStatus.UNKNOWN)

    def test_get_subsystem_status_unknown_for_unregistered(self):
        """get_subsystem_status returns UNKNOWN for unregistered subsystem."""
        status = self.manager.get_subsystem_status("nonexistent")
        self.assertEqual(status, SubsystemStatus.UNKNOWN)

    def test_on_subsystem_msg_updates_status(self):
        """_on_subsystem_msg sets subsystem to ONLINE."""
        self.manager.register_subsystem("simulator", "/velodyne_points", object)
        self.manager._on_subsystem_msg("simulator")
        self.assertTrue(self.manager.is_subsystem_online("simulator"))

    def test_check_health(self):
        """check_health detects timeout and updates status."""
        self.manager.register_subsystem("sim", "/topic", object, timeout=0.01)
        self.manager._on_subsystem_msg("sim")
        self.assertTrue(self.manager.is_subsystem_online("sim"))
        time.sleep(0.02)
        self.manager.check_health()
        self.assertFalse(self.manager.is_subsystem_online("sim"))

    def test_is_subsystem_online(self):
        """is_subsystem_online reflects current status."""
        self.manager.register_subsystem("nav", "/cmd_vel", object)
        self.assertFalse(self.manager.is_subsystem_online("nav"))
        self.manager._on_subsystem_msg("nav")
        self.assertTrue(self.manager.is_subsystem_online("nav"))

    def test_check_health_returns_dict(self):
        """check_health returns a dict mapping names to status strings."""
        self.manager.register_subsystem("sim", "/topic1", object)
        self.manager.register_subsystem("slam", "/topic2", object)
        health = self.manager.check_health()
        self.assertIn("sim", health)
        self.assertIn("slam", health)

    def test_get_all_online_false_when_any_offline(self):
        """get_all_online returns False when any subsystem is not online."""
        self.manager.register_subsystem("sim", "/topic1", object)
        self.manager.register_subsystem("slam", "/topic2", object)
        self.manager._on_subsystem_msg("sim")
        # Only sim is online, slam is not
        self.assertFalse(self.manager.get_all_online())

    def test_get_all_online_true_when_all_online(self):
        """get_all_online returns True when all subsystems are online."""
        self.manager.register_subsystem("sim", "/topic1", object)
        self.manager.register_subsystem("slam", "/topic2", object)
        self.manager._on_subsystem_msg("sim")
        self.manager._on_subsystem_msg("slam")
        self.assertTrue(self.manager.get_all_online())

    def test_get_summary(self):
        """get_summary returns dict with subsystem information."""
        self.manager.register_subsystem("sim", "/topic1", object)
        summary = self.manager.get_summary()
        self.assertIsInstance(summary, dict)

    def test_wait_for_subsystem_succeeds(self):
        """wait_for_subsystem returns True when subsystem comes online."""
        self.manager.register_subsystem("fast", "/topic", object)

        def activate():
            time.sleep(0.1)
            self.manager._on_subsystem_msg("fast")

        threading.Thread(target=activate, daemon=True).start()
        result = self.manager.wait_for_subsystem("fast", timeout=2.0, poll_interval=0.05)
        self.assertTrue(result)

    def test_wait_for_subsystem_times_out(self):
        """wait_for_subsystem returns False on timeout."""
        self.manager.register_subsystem("slow", "/topic", object)
        result = self.manager.wait_for_subsystem("slow", timeout=0.1, poll_interval=0.05)
        self.assertFalse(result)

    def test_wait_for_unregistered_subsystem(self):
        """wait_for_subsystem returns False for unregistered subsystem."""
        result = self.manager.wait_for_subsystem("ghost", timeout=0.1)
        self.assertFalse(result)


if __name__ == "__main__":
    unittest.main()
