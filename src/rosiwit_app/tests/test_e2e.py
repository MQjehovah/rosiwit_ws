"""End-to-end tests for rosiwit_app - simulating complete user workflows.

These tests simulate real user scenarios end-to-end:
1. System startup -> initialization -> ready
2. Mapping workflow -> save map -> restore position
3. Navigation workflow -> set waypoints -> navigate
4. Full lifecycle -> startup -> map -> navigate -> shutdown
"""

import json
import os
import shutil
import tempfile
import time
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

from rosiwit_app.app_node import AppState
from rosiwit_app.system_manager import SystemManager, SubsystemStatus
from rosiwit_app.map_manager import MapManager
from rosiwit_app.navigation_manager import NavigationManager, NavigationStatus
from rosiwit_app.waypoint_server import WaypointServer


class _FakeNode:
    """Minimal fake ROS2 Node for testing."""

    def __init__(self, name="fake"):
        self._name = name
        self._params = {}

    def get_logger(self):
        return _FakeLogger()

    def get_name(self):
        return self._name

    def declare_parameter(self, name, default_value=None):
        self._params[name] = default_value

    def get_parameter(self, name):
        return _FakeParam(self._params.get(name))

    def create_publisher(self, *a, **kw):
        return MagicMock()

    def create_subscription(self, *a, **kw):
        return MagicMock()

    def create_timer(self, *a, **kw):
        return MagicMock()

    def create_service(self, *a, **kw):
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
    def __init__(self, value=None):
        self._value = value

    def get_parameter_value(self):
        return self

    @property
    def string_value(self):
        return str(self._value) if self._value is not None else ""

    @property
    def double_value(self):
        return float(self._value) if self._value is not None else 0.0


# ===========================================================================
# E2E-001: Complete System Startup Flow
# ===========================================================================
class TestE2ESystemStartup(unittest.TestCase):
    """E2E test: System starts, registers subsystems, detects state correctly."""

    def test_full_startup_sequence(self):
        """E2E-001: Complete startup from initialization to monitoring."""
        node = _FakeNode()

        # Step 1: Create SystemManager and register subsystems
        sys_mgr = SystemManager(node)
        sys_mgr.register_subsystem("simulator", "/velodyne_points", object, timeout=5.0)
        sys_mgr.register_subsystem("slam", "/odom_estimated", object, timeout=5.0)
        sys_mgr.register_subsystem("navigation", "/cmd_vel", object, timeout=5.0)

        # Step 2: Initially all subsystems are offline
        health = sys_mgr.check_health()
        self.assertEqual(len(health), 3)
        for status in health.values():
            self.assertIn(status, [SubsystemStatus.OFFLINE.value, SubsystemStatus.UNKNOWN.value])

        # Step 3: Simulate subsystem messages coming online
        sys_mgr._on_subsystem_msg("simulator")
        self.assertTrue(sys_mgr.is_subsystem_online("simulator"))
        self.assertFalse(sys_mgr.is_subsystem_online("slam"))

        sys_mgr._on_subsystem_msg("slam")
        self.assertTrue(sys_mgr.is_subsystem_online("slam"))

        sys_mgr._on_subsystem_msg("navigation")
        self.assertTrue(sys_mgr.is_subsystem_online("navigation"))

        # Step 4: All subsystems online
        health = sys_mgr.check_health()
        for status in health.values():
            self.assertEqual(status, SubsystemStatus.ONLINE.value)

    def test_subsystem_timeout_during_operation(self):
        """E2E-002: Subsystem goes offline during operation (timeout)."""
        node = _FakeNode()
        sys_mgr = SystemManager(node)
        sys_mgr.register_subsystem("simulator", "/velodyne_points", object, timeout=0.05)

        # Bring online
        sys_mgr._on_subsystem_msg("simulator")
        self.assertTrue(sys_mgr.is_subsystem_online("simulator"))

        # Wait for timeout
        time.sleep(0.1)
        sys_mgr.check_health()
        self.assertFalse(sys_mgr.is_subsystem_online("simulator"))

    def test_subsystem_recovery(self):
        """E2E-003: Subsystem recovers after going offline."""
        node = _FakeNode()
        sys_mgr = SystemManager(node)
        sys_mgr.register_subsystem("simulator", "/velodyne_points", object, timeout=0.1)

        # Online -> timeout -> online again
        sys_mgr._on_subsystem_msg("simulator")
        self.assertTrue(sys_mgr.is_subsystem_online("simulator"))

        time.sleep(0.15)
        sys_mgr.check_health()
        self.assertFalse(sys_mgr.is_subsystem_online("simulator"))

        # Recovery
        sys_mgr._on_subsystem_msg("simulator")
        self.assertTrue(sys_mgr.is_subsystem_online("simulator"))


# ===========================================================================
# E2E-004: Complete Mapping Workflow
# ===========================================================================
class TestE2EMappingWorkflow(unittest.TestCase):
    """E2E test: Start -> detect no map -> save position -> detect map -> restore."""

    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.node = _FakeNode()

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_mapping_lifecycle(self):
        """E2E-004: Complete mapping lifecycle."""
        # Step 1: Create MapManager, no map exists
        mgr = MapManager(self.node, map_path=self.tmp)
        self.assertFalse(mgr.map_exists())

        # Step 2: Save current position
        mgr.save_position(x=1.5, y=2.5, z=0.0,
                          qx=0.0, qy=0.0, qz=0.0, qw=1.0)

        # Step 3: Simulate map creation (by external SLAM process)
        map_file = os.path.join(self.tmp, f"{mgr._map_file}.yaml")
        with open(map_file, "w") as f:
            f.write("image: test.pgm\nresolution: 0.05\n")
        self.assertTrue(mgr.map_exists())

        # Step 4: Verify position is still available
        pos = mgr.load_position()
        self.assertIsNotNone(pos)
        self.assertAlmostEqual(pos["x"], 1.5)
        self.assertAlmostEqual(pos["y"], 2.5)

        # Step 5: Get map server params for loading
        params = mgr.get_map_server_params()
        self.assertIn("yaml_filename", params)

    def test_map_overwrite_scenario(self):
        """E2E-005: Map gets overwritten (re-mapping)."""
        mgr = MapManager(self.node, map_path=self.tmp)

        # First map
        map_file = os.path.join(self.tmp, f"{mgr._map_file}.yaml")
        with open(map_file, "w") as f:
            f.write("image: old_map.pgm\nresolution: 0.05\n")
        self.assertTrue(mgr.map_exists())

        # Position from first session
        mgr.save_position(x=1.0, y=2.0, z=0.0,
                          qx=0.0, qy=0.0, qz=0.0, qw=1.0)

        # Overwrite map
        with open(map_file, "w") as f:
            f.write("image: new_map.pgm\nresolution: 0.05\n")

        # Position should still be from first session
        pos = mgr.load_position()
        self.assertAlmostEqual(pos["x"], 1.0)


# ===========================================================================
# E2E-006: Complete Waypoint Management Workflow
# ===========================================================================
class TestE2EWaypointWorkflow(unittest.TestCase):
    """E2E test: Define waypoints, save, reload, use for navigation."""

    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.wp_file = os.path.join(self.tmp, "waypoints.yaml")
        self.node = _FakeNode()

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_waypoint_lifecycle(self):
        """E2E-006: Complete waypoint lifecycle."""
        # Step 1: Create waypoint server with predefined waypoints
        server = WaypointServer(self.node, waypoints_file=self.wp_file)

        # Step 2: Add waypoints representing a patrol route
        waypoints = [
            ("home", 0.0, 0.0),
            ("corridor_a", 5.0, 0.0),
            ("room_1", 5.0, 3.0),
            ("room_2", 5.0, 6.0),
            ("corridor_b", 0.0, 6.0),
        ]
        for name, x, y in waypoints:
            server.add_waypoint(name, x=x, y=y)

        # Step 3: Verify all waypoints are available
        self.assertEqual(len(server.get_names()), 5)
        for name, x, y in waypoints:
            self.assertTrue(server.has_waypoint(name))
            pose = server.get_waypoint_as_pose_stamped(name)
            self.assertIsNotNone(pose)
            self.assertAlmostEqual(pose.pose.position.x, x)
            self.assertAlmostEqual(pose.pose.position.y, y)

        # Step 4: Remove a waypoint
        server.remove_waypoint("room_2")
        self.assertEqual(len(server.get_names()), 4)
        self.assertFalse(server.has_waypoint("room_2"))

    def test_waypoint_reload_persistence(self):
        """E2E-007: Waypoints survive application restart."""
        # Session 1: Create and add waypoints
        server1 = WaypointServer(self.node, waypoints_file=self.wp_file)
        server1.add_waypoint("start", x=0.0, y=0.0)
        server1.add_waypoint("end", x=10.0, y=10.0)
        server1.add_waypoint("checkpoint", x=5.0, y=5.0)

        # Session 2: Reload from file
        server2 = WaypointServer(self.node, waypoints_file=self.wp_file)
        self.assertEqual(len(server2.get_names()), 3)
        self.assertTrue(server2.has_waypoint("start"))
        self.assertTrue(server2.has_waypoint("end"))
        self.assertTrue(server2.has_waypoint("checkpoint"))

        # Verify coordinates are preserved
        end = server2.get_waypoint("end")
        self.assertAlmostEqual(end["x"], 10.0)
        self.assertAlmostEqual(end["y"], 10.0)

    def test_predefined_waypoints_config(self):
        """E2E-008: Load predefined waypoints from config/waypoints.yaml."""
        base = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        config_file = os.path.join(base, "config", "waypoints.yaml")
        if not os.path.isfile(config_file):
            self.skipTest("waypoints.yaml not found in config/")

        server = WaypointServer(self.node, waypoints_file=config_file)
        names = server.get_names()
        self.assertGreaterEqual(len(names), 2, "Should have at least 2 predefined waypoints")

        # Each waypoint should be convertible to PoseStamped
        for name in names:
            pose = server.get_waypoint_as_pose_stamped(name)
            self.assertIsNotNone(pose, f"Waypoint '{name}' should produce valid PoseStamped")


# ===========================================================================
# E2E-009: Navigation Status Workflow
# ===========================================================================
class TestE2ENavigationWorkflow(unittest.TestCase):
    """E2E test: Navigation request, in progress, complete."""

    @patch('rosiwit_app.navigation_manager.ActionClient')
    def test_navigation_status_lifecycle(self, mock_ac):
        """E2E-009: Navigation status transitions correctly."""
        node = _FakeNode()
        nav = NavigationManager(node, action_name="navigate_to_pose")

        # Step 1: Initially IDLE
        self.assertEqual(nav.status, NavigationStatus.IDLE)
        self.assertFalse(nav.is_navigating)

        # Step 2: Simulate goal accepted -> NAVIGATING
        nav._status = NavigationStatus.NAVIGATING
        self.assertTrue(nav.is_navigating)

        # Step 3: Simulate feedback during navigation
        nav._feedback_callback_internal(MagicMock())
        self.assertEqual(nav.status, NavigationStatus.NAVIGATING)

        # Step 4: Simulate goal succeeded
        mock_future = MagicMock()
        mock_result = MagicMock()
        mock_result.status = 4  # GOAL_STATUS_SUCCEEDED
        mock_future.result.return_value = mock_result
        nav._result_callback_internal(mock_future)
        self.assertEqual(nav.status, NavigationStatus.SUCCEEDED)
        self.assertFalse(nav.is_navigating)

    @patch('rosiwit_app.navigation_manager.ActionClient')
    def test_navigation_failed_status(self, mock_ac):
        """E2E-010: Navigation fails correctly."""
        node = _FakeNode()
        nav = NavigationManager(node, action_name="navigate_to_pose")

        mock_future = MagicMock()
        mock_result = MagicMock()
        mock_result.status = 6  # GOAL_STATUS_ABORTED (maps to FAILED)
        mock_future.result.return_value = mock_result
        nav._result_callback_internal(mock_future)
        self.assertEqual(nav.status, NavigationStatus.FAILED)

    @patch('rosiwit_app.navigation_manager.ActionClient')
    def test_navigation_canceled_status(self, mock_ac):
        """E2E-011: Navigation cancellation."""
        node = _FakeNode()
        nav = NavigationManager(node, action_name="navigate_to_pose")

        mock_future = MagicMock()
        mock_result = MagicMock()
        mock_result.status = 5  # GOAL_STATUS_CANCELED
        mock_future.result.return_value = mock_result
        nav._result_callback_internal(mock_future)
        self.assertEqual(nav.status, NavigationStatus.CANCELED)


# ===========================================================================
# E2E-012: Full Application Lifecycle
# ===========================================================================
class TestE2EFullLifecycle(unittest.TestCase):
    """E2E test: Complete application lifecycle from startup to shutdown."""

    def test_app_state_enum_completeness(self):
        """E2E-012: AppState defines all required lifecycle states."""
        required_states = ["INIT", "INITIALIZING", "READY", "NAVIGATING",
                           "MAPPING", "MAP_SAVING"]
        for state in required_states:
            self.assertTrue(hasattr(AppState, state))

    def test_system_summary_after_full_init(self):
        """E2E-013: System summary reflects full initialization state."""
        node = _FakeNode()
        sys_mgr = SystemManager(node)
        sys_mgr.register_subsystem("simulator", "/velodyne_points", object)
        sys_mgr.register_subsystem("slam", "/odom_estimated", object)
        sys_mgr.register_subsystem("navigation", "/cmd_vel", object)

        # Bring all online
        for name in ["simulator", "slam", "navigation"]:
            sys_mgr._on_subsystem_msg(name)

        # Check summary
        summary = sys_mgr.get_summary()
        self.assertEqual(len(summary), 3)
        for name, info in summary.items():
            self.assertIn("status", info)
            self.assertEqual(info["status"], SubsystemStatus.ONLINE.value)

    def test_map_manager_params_for_map_server(self):
        """E2E-014: Map manager provides correct params for map_server node."""
        node = _FakeNode()
        tmp = tempfile.mkdtemp()
        try:
            mgr = MapManager(node, map_path=tmp, map_file="test_map")
            params = mgr.get_map_server_params()
            self.assertIn("yaml_filename", params)
            self.assertTrue(params["yaml_filename"].endswith("test_map.yaml"))
        finally:
            shutil.rmtree(tmp)


# ===========================================================================
# E2E-015: Edge Cases and Error Handling
# ===========================================================================
class TestE2EEdgeCases(unittest.TestCase):
    """E2E edge case tests."""

    def test_load_position_when_no_file(self):
        """E2E-015: Loading position when no save file exists returns None."""
        node = _FakeNode()
        tmp = tempfile.mkdtemp()
        try:
            mgr = MapManager(node, map_path=tmp)
            pos = mgr.load_position()
            self.assertIsNone(pos)
        finally:
            shutil.rmtree(tmp)

    def test_nonexistent_subsystem_status(self):
        """E2E-016: Querying nonexistent subsystem returns gracefully."""
        node = _FakeNode()
        sys_mgr = SystemManager(node)
        status = sys_mgr.get_subsystem_status("nonexistent")
        self.assertIn(status, [SubsystemStatus.UNKNOWN, SubsystemStatus.OFFLINE])

    @patch('rosiwit_app.navigation_manager.ActionClient')
    def test_navigation_status_is_property(self, mock_ac):
        """E2E-017: Navigation status is a property, not a method."""
        nav = NavigationManager(_FakeNode(), action_name="navigate_to_pose")
        self.assertIsInstance(nav.status, NavigationStatus)

    def test_waypoint_get_nonexistent(self):
        """E2E-018: Getting nonexistent waypoint returns None."""
        tmp = tempfile.mkdtemp()
        try:
            server = WaypointServer(_FakeNode(),
                                     waypoints_file=os.path.join(tmp, "wp.yaml"))
            self.assertIsNone(server.get_waypoint("nonexistent"))
            self.assertIsNone(server.get_waypoint_as_pose_stamped("nonexistent"))
        finally:
            shutil.rmtree(tmp)


if __name__ == "__main__":
    unittest.main()
