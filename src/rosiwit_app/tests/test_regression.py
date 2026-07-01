"""Regression tests - one test per previously discovered bug to prevent recurrence.

Bug IDs are formatted as BUG-NNN with a descriptive name.
Each test reproduces the exact conditions that caused the original failure.
"""

import json
import os
import shutil
import tempfile
import unittest
from unittest.mock import MagicMock, patch

import sys
import types

# --- Mock nav2_msgs before importing source modules ---
if 'nav2_msgs' not in sys.modules:
    nav2_msgs = types.ModuleType('nav2_msgs')
    nav2_msgs_action = types.ModuleType('nav2_msgs.action')
    mock_navigate = MagicMock()
    mock_navigate.Goal = MagicMock
    mock_navigate.Result = MagicMock
    mock_navigate.Feedback = MagicMock
    nav2_msgs_action.NavigateToPose = mock_navigate
    nav2_msgs.action = nav2_msgs_action
    sys.modules['nav2_msgs'] = nav2_msgs
    sys.modules['nav2_msgs.action'] = nav2_msgs_action

from rosiwit_app.map_manager import MapManager
from rosiwit_app.system_manager import SystemManager, SubsystemStatus
from rosiwit_app.navigation_manager import NavigationManager, NavigationStatus
from rosiwit_app.waypoint_server import WaypointServer


class _FakeNode:
    def __init__(self, name="fake"):
        self._name = name

    def get_logger(self):
        return _FakeLogger()

    def get_name(self):
        return self._name

    def declare_parameter(self, name, default_value=None):
        pass

    def get_parameter(self, name):
        return _FakeParam()

    def create_publisher(self, *a, **kw):
        return MagicMock()

    def create_subscription(self, *a, **kw):
        return MagicMock()

    def create_service(self, *a, **kw):
        return MagicMock()

    def create_timer(self, *a, **kw):
        return MagicMock()

    def create_action_client(self, *a, **kw):
        return MagicMock()

    def destroy_timer(self, timer):
        pass


class _FakeLogger:
    def debug(self, *a, **kw): pass
    def info(self, *a, **kw): pass
    def warn(self, *a, **kw): pass
    def error(self, *a, **kw): pass
    def fatal(self, *a, **kw): pass


class _FakeParam:
    def get_parameter_value(self):
        return self

    @property
    def string_value(self):
        return ""

    @property
    def double_value(self):
        return 0.0

    @property
    def bool_value(self):
        return False


# ===========================================================================
# BUG-001: MapManager.save_position requires keyword arguments
# ===========================================================================
class TestBug001SavePositionKeywordOnly(unittest.TestCase):
    """save_position() uses keyword-only args (*, x, y, z, ...).

    Bug: Calling save_position(1.0, 2.0, ...) with positional args raised TypeError.
    Fix: Use keyword args save_position(x=1.0, y=2.0, ...).
    """

    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.node = _FakeNode()
        self.mgr = MapManager(self.node, map_path=self.tmp)

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_positional_args_raise_type_error(self):
        """BUG-001: Positional args to save_position raise TypeError."""
        with self.assertRaises(TypeError):
            self.mgr.save_position(1.0, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0)

    def test_keyword_args_succeed(self):
        """BUG-001: Keyword args to save_position work correctly."""
        result = self.mgr.save_position(x=1.0, y=2.0, z=0.0,
                                         qx=0.0, qy=0.0, qz=0.0, qw=1.0)
        self.assertTrue(result)

    def test_negative_coords_keyword(self):
        """BUG-001: Negative coordinates work with keyword args."""
        result = self.mgr.save_position(x=-5.5, y=-3.2, z=-0.1,
                                         qx=0.0, qy=0.0, qz=0.707, qw=0.707)
        self.assertTrue(result)
        data = self.mgr.load_position()
        self.assertAlmostEqual(data["x"], -5.5)


# ===========================================================================
# BUG-002: SystemManager doesn't have record_activity method
# ===========================================================================
class TestBug002NoRecordActivity(unittest.TestCase):
    """SystemManager uses _on_subsystem_msg, not record_activity.

    Bug: Tests called non-existent record_activity() method.
    Fix: Use _on_subsystem_msg() or subscription callback instead.
    """

    def test_no_record_activity_method(self):
        """BUG-002: SystemManager should not have record_activity."""
        mgr = SystemManager(_FakeNode())
        self.assertFalse(hasattr(mgr, 'record_activity'),
                         "record_activity should not exist on SystemManager")

    def test_on_subsystem_msg_exists(self):
        """BUG-002: SystemManager has _on_subsystem_msg instead."""
        mgr = SystemManager(_FakeNode())
        self.assertTrue(hasattr(mgr, '_on_subsystem_msg'),
                        "_on_subsystem_msg should exist on SystemManager")

    def test_on_subsystem_msg_brings_online(self):
        """BUG-002: _on_subsystem_msg correctly marks subsystem as online."""
        mgr = SystemManager(_FakeNode())
        mgr.register_subsystem("test", "/topic", object)
        mgr._on_subsystem_msg("test")
        self.assertTrue(mgr.is_subsystem_online("test"))


# ===========================================================================
# BUG-003: SystemManager doesn't have get_subsystem_info method
# ===========================================================================
class TestBug003NoGetSubsystemInfo(unittest.TestCase):
    """SystemManager uses get_subsystem_status, not get_subsystem_info.

    Bug: Tests called non-existent get_subsystem_info() method.
    Fix: Use get_subsystem_status() instead.
    """

    def test_no_get_subsystem_info(self):
        """BUG-003: get_subsystem_info should not exist."""
        mgr = SystemManager(_FakeNode())
        self.assertFalse(hasattr(mgr, 'get_subsystem_info'),
                         "get_subsystem_info should not exist")

    def test_get_subsystem_status_exists(self):
        """BUG-003: get_subsystem_status exists and works."""
        mgr = SystemManager(_FakeNode())
        mgr.register_subsystem("test", "/topic", object)
        status = mgr.get_subsystem_status("test")
        self.assertEqual(status, SubsystemStatus.UNKNOWN)


# ===========================================================================
# BUG-004: WaypointServer constructor parameter is waypoints_file
# ===========================================================================
class TestBug004WaypointServerParam(unittest.TestCase):
    """WaypointServer constructor uses waypoints_file, not waypoint_file.

    Bug: Creating WaypointServer with waypoint_file=... caused TypeError.
    Fix: Use waypoints_file=... parameter name.
    """

    def test_waypoint_file_param_rejected(self):
        """BUG-004: waypoint_file parameter should cause TypeError."""
        tmp = tempfile.mkdtemp()
        try:
            with self.assertRaises(TypeError):
                WaypointServer(_FakeNode(), waypoint_file=os.path.join(tmp, "wp.yaml"))
        finally:
            shutil.rmtree(tmp)

    def test_waypoints_file_param_accepted(self):
        """BUG-004: waypoints_file parameter works correctly."""
        tmp = tempfile.mkdtemp()
        try:
            server = WaypointServer(_FakeNode(), waypoints_file=os.path.join(tmp, "wp.yaml"))
            self.assertIsNotNone(server)
        finally:
            shutil.rmtree(tmp)


# ===========================================================================
# BUG-005: WaypointServer has get_waypoint_as_pose_stamped
# ===========================================================================
class TestBug005WaypointPoseStamped(unittest.TestCase):
    """WaypointServer uses get_waypoint_as_pose_stamped.

    Bug: Calling get_waypoint_pose_stamped() raised AttributeError.
    Fix: Use get_waypoint_as_pose_stamped() instead.
    """

    def test_old_method_not_exists(self):
        """BUG-005: get_waypoint_pose_stamped should not exist."""
        tmp = tempfile.mkdtemp()
        try:
            server = WaypointServer(_FakeNode(), waypoints_file=os.path.join(tmp, "wp.yaml"))
            self.assertFalse(hasattr(server, 'get_waypoint_pose_stamped'))
        finally:
            shutil.rmtree(tmp)

    def test_correct_method_exists(self):
        """BUG-005: get_waypoint_as_pose_stamped exists and works."""
        tmp = tempfile.mkdtemp()
        try:
            server = WaypointServer(_FakeNode(), waypoints_file=os.path.join(tmp, "wp.yaml"))
            self.assertTrue(hasattr(server, 'get_waypoint_as_pose_stamped'))
            server.add_waypoint("test", x=1.0, y=2.0)
            pose = server.get_waypoint_as_pose_stamped("test")
            self.assertIsNotNone(pose)
        finally:
            shutil.rmtree(tmp)


# ===========================================================================
# BUG-006: NavigationManager uses _feedback_callback_internal / _result_callback_internal
# ===========================================================================
class TestBug006NavManagerCallbacks(unittest.TestCase):
    """NavigationManager uses specific internal callback names.

    Bug: Tests used _on_feedback / _on_result which don't exist.
    Fix: Use _feedback_callback_internal / _result_callback_internal.
    """

    @patch('rosiwit_app.navigation_manager.ActionClient')
    def test_old_callbacks_not_exists(self, mock_ac):
        """BUG-006: Old callback names should not exist."""
        nav = NavigationManager(_FakeNode(), action_name="navigate_to_pose")
        self.assertFalse(hasattr(nav, '_on_feedback'))
        self.assertFalse(hasattr(nav, '_on_result'))

    @patch('rosiwit_app.navigation_manager.ActionClient')
    def test_correct_callbacks_exist(self, mock_ac):
        """BUG-006: Correct callback names exist."""
        nav = NavigationManager(_FakeNode(), action_name="navigate_to_pose")
        self.assertTrue(hasattr(nav, '_feedback_callback_internal'))
        self.assertTrue(hasattr(nav, '_result_callback_internal'))

    @patch('rosiwit_app.navigation_manager.ActionClient')
    def test_result_callback_with_succeeded(self, mock_ac):
        """BUG-006: _result_callback_internal sets SUCCEEDED on status 4."""
        nav = NavigationManager(_FakeNode(), action_name="navigate_to_pose")
        mock_future = MagicMock()
        mock_result = MagicMock()
        mock_result.status = 4  # GOAL_STATUS_SUCCEEDED
        mock_future.result.return_value = mock_result
        nav._result_callback_internal(mock_future)
        self.assertEqual(nav.status, NavigationStatus.SUCCEEDED)

    @patch('rosiwit_app.navigation_manager.ActionClient')
    def test_result_callback_with_failed(self, mock_ac):
        """BUG-006: _result_callback_internal sets FAILED on other statuses."""
        nav = NavigationManager(_FakeNode(), action_name="navigate_to_pose")
        mock_future = MagicMock()
        mock_result = MagicMock()
        mock_result.status = 6  # GOAL_STATUS_ABORTED (maps to FAILED)
        mock_future.result.return_value = mock_result
        nav._result_callback_internal(mock_future)
        self.assertEqual(nav.status, NavigationStatus.FAILED)


# ===========================================================================
# BUG-007: MapManager uses map_exists(), not map_files_exist()
# ===========================================================================
class TestBug007MapExists(unittest.TestCase):
    """MapManager uses map_exists(), not map_files_exist().

    Bug: Calling map_files_exist() raised AttributeError.
    Fix: Use map_exists() instead.
    """

    def test_map_files_exist_not_exists(self):
        """BUG-007: map_files_exist should not exist."""
        mgr = MapManager(_FakeNode(), map_path=tempfile.mkdtemp())
        self.assertFalse(hasattr(mgr, 'map_files_exist'))

    def test_map_exists_works(self):
        """BUG-007: map_exists works correctly."""
        tmp = tempfile.mkdtemp()
        try:
            mgr = MapManager(_FakeNode(), map_path=tmp)
            self.assertFalse(mgr.map_exists())
            # Create yaml file
            with open(os.path.join(tmp, f"{mgr._map_file}.yaml"), "w") as f:
                f.write("image: test.pgm\nresolution: 0.05\n")
            self.assertTrue(mgr.map_exists())
        finally:
            shutil.rmtree(tmp)


if __name__ == "__main__":
    unittest.main()
