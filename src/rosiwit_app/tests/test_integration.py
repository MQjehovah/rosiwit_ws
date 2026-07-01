"""Integration tests for rosiwit_app module cooperation.

Tests that modules work together correctly, including:
- MapManager + WaypointServer coordinate data
- SystemManager + NavigationManager lifecycle
- AppNode state machine with mocked ROS2 infrastructure

These tests do NOT require a running ROS2 system - they mock
the ROS2 communication layer and test module-level integration.
"""

import json
import os
import shutil
import sys
import tempfile
import time
import types
import unittest
from unittest.mock import MagicMock, patch

# --- Mock nav2_msgs before importing ---
if 'nav2_msgs' not in sys.modules:
    nav2_msgs = types.ModuleType('nav2_msgs')
    nav2_msgs_action = types.ModuleType('nav2_msgs.action')
    mock_navigate = type('NavigateToPose', (), {
        'Goal': type('Goal', (), {}),
        'Result': type('Result', (), {}),
        'Feedback': type('Feedback', (), {}),
    })
    nav2_msgs_action.NavigateToPose = mock_navigate
    nav2_msgs.action = nav2_msgs_action
    sys.modules['nav2_msgs'] = nav2_msgs
    sys.modules['nav2_msgs.action'] = nav2_msgs_action

from rosiwit_app.map_manager import MapManager
from rosiwit_app.waypoint_server import WaypointServer
from rosiwit_app.system_manager import SystemManager, SubsystemStatus
from rosiwit_app.navigation_manager import NavigationManager, NavigationStatus
from rosiwit_app.app_node import AppState


class _FakeClock:
    class _FakeTime:
        def to_msg(self):
            from builtin_interfaces.msg import Time
            return Time(sec=0, nanosec=0)
    def now(self):
        return self._FakeTime()


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

    def create_client(self, srv_type, name):
        return MagicMock()

    def create_subscription(self, msg_type, topic, callback, qos_profile=10):
        return MagicMock()

    def get_clock(self):
        return _FakeClock()


class TestMapManagerWaypointServerIntegration(unittest.TestCase):
    """Test that MapManager and WaypointServer work together.

    Scenario: After building a map and saving the position, the user
    defines a waypoint near the saved position.
    """

    def setUp(self):
        self.fake_node = _FakeNode()
        self.tmpdir = tempfile.mkdtemp()

    def tearDown(self):
        shutil.rmtree(self.tmpdir, ignore_errors=True)

    def test_save_position_then_create_waypoint(self):
        """Position saved by MapManager can be used to create a Waypoint."""
        # Save current position using MapManager
        map_mgr = MapManager(self.fake_node, map_path=self.tmpdir, map_file="test")
        pos = {"x": 1.5, "y": 2.5, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0}
        map_mgr.save_position(pos)

        # Load position and create a waypoint
        loaded_pos = map_mgr.load_position()
        self.assertIsNotNone(loaded_pos)

        wp_file = os.path.join(self.tmpdir, "waypoints.yaml")
        wp_server = WaypointServer(self.fake_node, waypoints_file=wp_file)
        wp_server.add_waypoint(
            "saved_pos",
            x=loaded_pos["x"], y=loaded_pos["y"], z=loaded_pos["z"],
            qx=loaded_pos["qx"], qy=loaded_pos["qy"], qz=loaded_pos["qz"], qw=loaded_pos["qw"],
            frame_id="map", description="Position from map_manager"
        )

        # Reload and verify round-trip
        wp_server2 = WaypointServer(self.fake_node, waypoints_file=wp_file)
        self.assertTrue(wp_server2.has_waypoint("saved_pos"))
        pose = wp_server2.get_waypoint_as_pose_stamped("saved_pos")
        self.assertAlmostEqual(pose.pose.position.x, 1.5)
        self.assertAlmostEqual(pose.pose.position.y, 2.5)

    def test_map_exists_reflects_saved_map(self):
        """MapManager correctly detects when SLAM has saved map files."""
        map_mgr = MapManager(self.fake_node, map_path=self.tmpdir, map_file="fast_lio2_map")
        self.assertFalse(map_mgr.map_exists())

        # Simulate SLAM saving a map
        yaml_path = os.path.join(self.tmpdir, "fast_lio2_map.yaml")
        with open(yaml_path, "w") as f:
            f.write("image: fast_lio2_map.pgm\nresolution: 0.05\n")

        self.assertTrue(map_mgr.map_exists())


class TestSystemManagerNavigationManagerIntegration(unittest.TestCase):
    """Test SystemManager and NavigationManager cooperation."""

    def setUp(self):
        self.fake_node = _FakeNode()

    @patch('rosiwit_app.navigation_manager.ActionClient')
    def test_navigation_starts_when_subsystem_online(self, mock_action_client_cls):
        """NavigationManager transitions correctly when subsystem is ready."""
        sys_mgr = SystemManager(self.fake_node)
        sys_mgr.register_subsystem("navigation", "/navigate_to_pose", object)

        nav_mgr = NavigationManager(self.fake_node, action_name="/navigate_to_pose")

        # Initially not online, not navigating
        self.assertFalse(sys_mgr.is_subsystem_online("navigation"))
        self.assertFalse(nav_mgr.is_navigating)

        # Simulate navigation coming online via the internal callback
        sys_mgr._on_subsystem_msg("navigation")
        self.assertTrue(sys_mgr.is_subsystem_online("navigation"))

    @patch('rosiwit_app.navigation_manager.ActionClient')
    def test_timeout_propagation(self, mock_action_client_cls):
        """Subsystem timeout and navigation timeout are independent."""
        sys_mgr = SystemManager(self.fake_node)
        sys_mgr.register_subsystem("nav", "/topic", object, timeout=0.05)
        sys_mgr._on_subsystem_msg("nav")

        nav_mgr = NavigationManager(self.fake_node, action_name="/navigate_to_pose")

        # Wait for subsystem to time out
        time.sleep(0.1)
        sys_mgr.check_health()
        self.assertEqual(
            sys_mgr.get_subsystem_status("nav"),
            SubsystemStatus.OFFLINE
        )
        # Navigation manager is still IDLE (not affected)
        self.assertEqual(nav_mgr.status, NavigationStatus.IDLE)


class TestAppStateTransitions(unittest.TestCase):
    """Test AppState enum values match the expected state machine."""

    def test_all_transitions_have_states(self):
        """All states in the state machine diagram exist."""
        required = [
            AppState.INIT,
            AppState.INITIALIZING,
            AppState.READY,
            AppState.NAVIGATING,
            AppState.MAPPING,
            AppState.MAP_SAVING,
        ]
        for state in required:
            self.assertIsNotNone(state)

    def test_error_state_exists(self):
        """ERROR state exists as a recovery target."""
        self.assertIsNotNone(AppState.ERROR)


class TestNavigationManagerStatusIntegration(unittest.TestCase):
    """Test NavigationManager status lifecycle."""

    @patch('rosiwit_app.navigation_manager.ActionClient')
    def test_full_navigation_lifecycle(self, mock_action_client_cls):
        """NavigationManager goes through complete lifecycle:
        IDLE → NAVIGATING → SUCCEEDED.
        """
        mgr = NavigationManager(_FakeNode(), action_name="/navigate_to_pose")
        self.assertEqual(mgr.status, NavigationStatus.IDLE)

        # Simulate navigation start
        mgr._status = NavigationStatus.NAVIGATING
        mgr._start_time = time.time()
        self.assertTrue(mgr.is_navigating)

        # Simulate success result
        mock_future = MagicMock()
        mock_result = MagicMock()
        mock_result.status = 4  # SUCCEEDED
        mock_future.result.return_value = mock_result

        mgr._get_result_callback(mock_future)
        self.assertEqual(mgr.status, NavigationStatus.SUCCEEDED)

    @patch('rosiwit_app.navigation_manager.ActionClient')
    def test_navigation_failure_lifecycle(self, mock_action_client_cls):
        """NavigationManager handles failure: IDLE → NAVIGATING → FAILED."""
        mgr = NavigationManager(_FakeNode(), action_name="/navigate_to_pose")
        mgr._status = NavigationStatus.NAVIGATING
        mgr._start_time = time.time()

        mock_future = MagicMock()
        mock_result = MagicMock()
        mock_result.status = 99  # Unknown → FAILED
        mock_future.result.return_value = mock_result

        mgr._get_result_callback(mock_future)
        self.assertEqual(mgr.status, NavigationStatus.FAILED)

    @patch('rosiwit_app.navigation_manager.ActionClient')
    def test_navigation_cancel_lifecycle(self, mock_action_client_cls):
        """NavigationManager handles cancellation."""
        mgr = NavigationManager(_FakeNode(), action_name="/navigate_to_pose")
        mgr._status = NavigationStatus.NAVIGATING
        mgr._goal_handle = MagicMock()

        mock_future = MagicMock()
        mock_result = MagicMock()
        mock_result.status = 5  # CANCELED
        mock_future.result.return_value = mock_result

        mgr._get_result_callback(mock_future)
        self.assertEqual(mgr.status, NavigationStatus.CANCELED)

    @patch('rosiwit_app.navigation_manager.ActionClient')
    def test_navigation_timeout_lifecycle(self, mock_action_client_cls):
        """NavigationManager handles timeout."""
        mgr = NavigationManager(_FakeNode(), action_name="/navigate_to_pose", timeout=0.01)
        mgr._status = NavigationStatus.NAVIGATING
        mgr._start_time = time.time() - 1.0  # Started 1 sec ago
        mgr._goal_handle = MagicMock()

        mgr.check_timeout()
        self.assertEqual(mgr.status, NavigationStatus.TIMED_OUT)


class TestWaypointServerMapPathIntegration(unittest.TestCase):
    """Test that waypoints and map data share the same coordinate frame."""

    def test_waypoints_use_map_frame(self):
        """Waypoints loaded from project config use 'map' frame."""
        config_path = os.path.join(
            os.path.dirname(__file__), "..", "config", "waypoints.yaml"
        )
        if not os.path.isfile(config_path):
            self.skipTest("waypoints.yaml not found")

        fake_node = _FakeNode()
        ws = WaypointServer(fake_node, waypoints_file=config_path)
        for name in ws.get_names():
            pose = ws.get_waypoint_as_pose_stamped(name)
            self.assertEqual(
                pose.header.frame_id, "map",
                f"Waypoint '{name}' should use 'map' frame, got '{pose.header.frame_id}'"
            )


class TestMapManagerEdgeCases(unittest.TestCase):
    """Edge case tests for MapManager."""

    def setUp(self):
        self.fake_node = _FakeNode()
        self.tmpdir = tempfile.mkdtemp()

    def tearDown(self):
        shutil.rmtree(self.tmpdir, ignore_errors=True)

    def test_concurrent_save_position(self):
        """Multiple rapid saves don't corrupt the JSON file."""
        mgr = MapManager(self.fake_node, map_path=self.tmpdir, map_file="test")
        for i in range(50):
            mgr.save_position({"x": float(i), "y": float(i), "z": 0.0,
                               "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0})
        pos = mgr.load_position()
        self.assertIsNotNone(pos)
        self.assertEqual(pos["x"], 49.0)

    def test_map_exists_with_multiple_map_files(self):
        """map_exists returns True when correct map file exists among others."""
        with open(os.path.join(self.tmpdir, "other_map.yaml"), "w") as f:
            f.write("data: test\n")
        mgr = MapManager(self.fake_node, map_path=self.tmpdir, map_file="fast_lio2_map")
        self.assertFalse(mgr.map_exists())

        with open(os.path.join(self.tmpdir, "fast_lio2_map.yaml"), "w") as f:
            f.write("image: fast_lio2_map.pgm\n")
        self.assertTrue(mgr.map_exists())


class TestWaypointServerEdgeCases(unittest.TestCase):
    """Edge case tests for WaypointServer."""

    def setUp(self):
        self.fake_node = _FakeNode()
        self.tmpdir = tempfile.mkdtemp()
        self.wp_file = os.path.join(self.tmpdir, "waypoints.yaml")
        self.ws = WaypointServer(self.fake_node, waypoints_file=self.wp_file)

    def tearDown(self):
        shutil.rmtree(self.tmpdir, ignore_errors=True)

    def test_overwrite_existing_waypoint(self):
        """Adding a waypoint with existing name overwrites it."""
        self.ws.add_waypoint("dup", x=1.0, y=0.0, z=0.0,
                             qx=0.0, qy=0.0, qz=0.0, qw=1.0)
        self.ws.add_waypoint("dup", x=5.0, y=5.0, z=0.0,
                             qx=0.0, qy=0.0, qz=0.0, qw=1.0)
        pose = self.ws.get_waypoint_as_pose_stamped("dup")
        self.assertAlmostEqual(pose.pose.position.x, 5.0)

    def test_special_characters_in_name(self):
        """Waypoint name with special characters works."""
        self.ws.add_waypoint("point-1_v2", x=0.0, y=0.0, z=0.0,
                             qx=0.0, qy=0.0, qz=0.0, qw=1.0)
        self.assertTrue(self.ws.has_waypoint("point-1_v2"))

    def test_large_number_of_waypoints(self):
        """WaypointServer handles large number of waypoints."""
        for i in range(100):
            self.ws.add_waypoint(f"wp_{i}", x=float(i), y=0.0, z=0.0,
                                 qx=0.0, qy=0.0, qz=0.0, qw=1.0)
        self.assertEqual(len(self.ws.list_waypoints()), 100)

        ws2 = WaypointServer(self.fake_node, waypoints_file=self.wp_file)
        self.assertEqual(len(ws2.list_waypoints()), 100)


if __name__ == "__main__":
    unittest.main()
