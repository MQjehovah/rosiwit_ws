"""Unit tests for NavigationManager.

Tests navigation state transitions, callback handling,
cancel behavior, timeout detection, and summary reporting.
All ROS2 action client calls are mocked.
"""

import time
import unittest
from unittest.mock import MagicMock, patch, PropertyMock

from rosiwit_app.navigation_manager import NavigationManager, NavigationStatus


class _FakeNode:
    """Minimal fake ROS2 node for testing."""

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


def _create_mock_navigation_manager():
    """Create NavigationManager with mocked action client."""
    fake_node = _FakeNode()

    with patch('rosiwit_app.navigation_manager.ActionClient') as MockActionClient:
        mock_ac = MagicMock()
        mock_ac.wait_for_server.return_value = True
        mock_ac.server_is_ready.return_value = True
        MockActionClient.return_value = mock_ac

        mgr = NavigationManager(fake_node, action_name="navigate_to_pose", timeout=120.0)
    return mgr, mock_ac


class TestNavigationManagerInit(unittest.TestCase):
    """Test initialization and initial state."""

    def test_initial_status_is_idle(self):
        """New NavigationManager has IDLE status."""
        mgr, _ = _create_mock_navigation_manager()
        self.assertEqual(mgr.status, NavigationStatus.IDLE)

    def test_initial_not_navigating(self):
        """New NavigationManager is not navigating."""
        mgr, _ = _create_mock_navigation_manager()
        self.assertFalse(mgr.is_navigating)

    def test_initial_no_goal(self):
        """New NavigationManager has no current goal."""
        mgr, _ = _create_mock_navigation_manager()
        self.assertIsNone(mgr.current_goal)


class TestNavigationManagerCallbacks(unittest.TestCase):
    """Test callback registration and invocation."""

    def test_set_feedback_callback(self):
        """set_callbacks stores the feedback callback."""
        mgr, _ = _create_mock_navigation_manager()
        cb = MagicMock()
        mgr.set_callbacks(feedback_cb=cb)
        # Internal callback should be stored
        self.assertEqual(mgr._feedback_callback, cb)

    def test_set_result_callback(self):
        """set_callbacks stores the result callback."""
        mgr, _ = _create_mock_navigation_manager()
        cb = MagicMock()
        mgr.set_callbacks(result_cb=cb)
        self.assertEqual(mgr._result_callback, cb)

    def test_feedback_callback_delegates(self):
        """_feedback_callback_internal delegates to user callback."""
        mgr, _ = _create_mock_navigation_manager()
        user_cb = MagicMock()
        mgr.set_callbacks(feedback_cb=user_cb)

        fake_feedback_msg = MagicMock()
        fake_feedback_msg.feedback = "feedback_data"
        mgr._feedback_callback_internal(fake_feedback_msg)
        user_cb.assert_called_once_with("feedback_data")

    def test_feedback_callback_error_caught(self):
        """_feedback_callback_internal catches user callback exceptions."""
        mgr, _ = _create_mock_navigation_manager()
        def bad_cb(fb):
            raise ValueError("test error")
        mgr.set_callbacks(feedback_cb=bad_cb)

        fake_feedback_msg = MagicMock()
        fake_feedback_msg.feedback = "data"
        # Should not raise
        mgr._feedback_callback_internal(fake_feedback_msg)


class TestNavigationManagerSendGoal(unittest.TestCase):
    """Test send_goal behavior."""

    def test_send_goal_when_idle(self):
        """send_goal returns True when status is IDLE."""
        mgr, mock_ac = _create_mock_navigation_manager()

        mock_future = MagicMock()
        mock_ac.send_goal_async.return_value = mock_future

        pose = MagicMock()
        pose.header.frame_id = "map"
        pose.pose.position.x = 1.0
        pose.pose.position.y = 2.0

        result = mgr.send_goal(pose)
        self.assertTrue(result)
        self.assertEqual(mgr.status, NavigationStatus.NAVIGATING)
        self.assertTrue(mgr.is_navigating)
        mock_ac.send_goal_async.assert_called_once()

    def test_send_goal_when_navigating_fails(self):
        """send_goal returns False when already navigating."""
        mgr, mock_ac = _create_mock_navigation_manager()
        mgr._status = NavigationStatus.NAVIGATING

        pose = MagicMock()
        result = mgr.send_goal(pose)
        self.assertFalse(result)
        mock_ac.send_goal_async.assert_not_called()

    def test_send_goal_stores_current_goal(self):
        """send_goal stores the pose as current_goal."""
        mgr, mock_ac = _create_mock_navigation_manager()
        mock_future = MagicMock()
        mock_ac.send_goal_async.return_value = mock_future

        pose = MagicMock()
        mgr.send_goal(pose)
        self.assertEqual(mgr.current_goal, pose)


class TestNavigationManagerGoalResponse(unittest.TestCase):
    """Test _goal_response_callback behavior."""

    def test_goal_accepted(self):
        """Accepted goal transitions to waiting for result."""
        mgr, mock_ac = _create_mock_navigation_manager()
        mgr._status = NavigationStatus.NAVIGATING

        mock_goal_handle = MagicMock()
        mock_goal_handle.accepted = True
        mock_result_future = MagicMock()
        mock_goal_handle.get_result_async.return_value = mock_result_future

        future = MagicMock()
        future.result.return_value = mock_goal_handle

        mgr._goal_response_callback(future)
        self.assertEqual(mgr._goal_handle, mock_goal_handle)

    def test_goal_rejected(self):
        """Rejected goal sets status to FAILED."""
        mgr, _ = _create_mock_navigation_manager()
        mgr._status = NavigationStatus.NAVIGATING

        mock_goal_handle = MagicMock()
        mock_goal_handle.accepted = False

        future = MagicMock()
        future.result.return_value = mock_goal_handle

        mgr._goal_response_callback(future)
        self.assertEqual(mgr.status, NavigationStatus.FAILED)


class TestNavigationManagerResultCallback(unittest.TestCase):
    """Test _result_callback_internal behavior."""

    def test_result_succeeded(self):
        """Status 4 sets SUCCEEDED status."""
        mgr, _ = _create_mock_navigation_manager()
        mgr._status = NavigationStatus.NAVIGATING
        mgr._start_time = time.time()

        future = MagicMock()
        result = MagicMock()
        result.status = 4  # GOAL_STATUS_SUCCEEDED
        future.result.return_value = result

        mgr._result_callback_internal(future)
        self.assertEqual(mgr.status, NavigationStatus.SUCCEEDED)

    def test_result_canceled(self):
        """Status 5 sets CANCELED status."""
        mgr, _ = _create_mock_navigation_manager()
        mgr._status = NavigationStatus.NAVIGATING

        future = MagicMock()
        result = MagicMock()
        result.status = 5  # GOAL_STATUS_CANCELED
        future.result.return_value = result

        mgr._result_callback_internal(future)
        self.assertEqual(mgr.status, NavigationStatus.CANCELED)

    def test_result_failed_other(self):
        """Other status codes set FAILED status."""
        mgr, _ = _create_mock_navigation_manager()
        mgr._status = NavigationStatus.NAVIGATING

        future = MagicMock()
        result = MagicMock()
        result.status = 3  # GOAL_STATUS_ABORTED or other
        future.result.return_value = result

        mgr._result_callback_internal(future)
        self.assertEqual(mgr.status, NavigationStatus.FAILED)

    def test_result_callback_notifies_user(self):
        """Result callback invokes user result_callback."""
        mgr, _ = _create_mock_navigation_manager()
        mgr._status = NavigationStatus.NAVIGATING

        user_result_cb = MagicMock()
        mgr.set_callbacks(result_cb=user_result_cb)

        future = MagicMock()
        result = MagicMock()
        result.status = 4
        future.result.return_value = result

        mgr._result_callback_internal(future)
        user_result_cb.assert_called_once_with(NavigationStatus.SUCCEEDED)


class TestNavigationManagerCancel(unittest.TestCase):
    """Test cancel_goal behavior."""

    def test_cancel_when_navigating(self):
        """cancel_goal returns True when navigating."""
        mgr, mock_ac = _create_mock_navigation_manager()
        mgr._status = NavigationStatus.NAVIGATING
        mgr._goal_handle = MagicMock()

        mock_cancel_future = MagicMock()
        mgr._goal_handle.cancel_goal_async.return_value = mock_cancel_future

        result = mgr.cancel_goal()
        self.assertTrue(result)

    def test_cancel_when_idle(self):
        """cancel_goal returns False when not navigating."""
        mgr, _ = _create_mock_navigation_manager()
        result = mgr.cancel_goal()
        self.assertFalse(result)


class TestNavigationManagerTimeout(unittest.TestCase):
    """Test check_timeout behavior."""

    def test_check_timeout_not_timed_out(self):
        """check_timeout returns False when within limit."""
        mgr, _ = _create_mock_navigation_manager()
        mgr._status = NavigationStatus.NAVIGATING
        mgr._start_time = time.time()
        mgr._timeout = 120.0
        self.assertFalse(mgr.check_timeout())

    def test_check_timeout_when_not_navigating(self):
        """check_timeout returns False when not navigating."""
        mgr, _ = _create_mock_navigation_manager()
        self.assertFalse(mgr.check_timeout())

    def test_check_timeout_timed_out(self):
        """check_timeout returns True and sets TIMED_OUT when elapsed."""
        mgr, mock_ac = _create_mock_navigation_manager()
        mgr._status = NavigationStatus.NAVIGATING
        mgr._start_time = time.time() - 200.0  # Way past timeout
        mgr._timeout = 120.0
        mgr._goal_handle = MagicMock()
        mock_cancel_future = MagicMock()
        mgr._goal_handle.cancel_goal_async.return_value = mock_cancel_future

        result = mgr.check_timeout()
        self.assertTrue(result)
        self.assertEqual(mgr.status, NavigationStatus.TIMED_OUT)


class TestNavigationManagerSummary(unittest.TestCase):
    """Test get_summary method."""

    def test_summary_idle(self):
        """Summary shows idle state correctly."""
        mgr, _ = _create_mock_navigation_manager()
        summary = mgr.get_summary()
        self.assertEqual(summary["status"], "idle")
        self.assertFalse(summary["server_ready"])  # mock returns True, but we override

    def test_summary_with_goal(self):
        """Summary includes goal details when present."""
        mgr, _ = _create_mock_navigation_manager()
        pose = MagicMock()
        pose.header.frame_id = "map"
        pose.pose.position.x = 1.0
        pose.pose.position.y = 2.0
        mgr._current_goal = pose
        mgr._status = NavigationStatus.NAVIGATING
        mgr._start_time = time.time()

        summary = mgr.get_summary()
        self.assertEqual(summary["status"], "navigating")
        self.assertIn("goal", summary)
        self.assertEqual(summary["goal"]["frame"], "map")
        self.assertIn("elapsed", summary)


class TestNavigationManagerWaitForServer(unittest.TestCase):
    """Test wait_for_server."""

    def test_wait_for_server_ready(self):
        """wait_for_server returns True when server is ready."""
        mgr, mock_ac = _create_mock_navigation_manager()
        mock_ac.wait_for_server.return_value = True
        self.assertTrue(mgr.wait_for_server(timeout=5.0))

    def test_wait_for_server_not_ready(self):
        """wait_for_server returns False when server not ready."""
        mgr, mock_ac = _create_mock_navigation_manager()
        mock_ac.wait_for_server.return_value = False
        self.assertFalse(mgr.wait_for_server(timeout=1.0))


if __name__ == "__main__":
    unittest.main()
