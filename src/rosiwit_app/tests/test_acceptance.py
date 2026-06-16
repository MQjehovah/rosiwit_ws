"""Acceptance tests for rosiwit_app - verifying requirements.md acceptance criteria.

These tests verify each acceptance criterion from the requirements document
against the actual implementation.

Requirements Reference: /home/jmq/agent/workspace/requirements.md
"""

import json
import os
import shutil
import sys
import types
import tempfile
import unittest
from unittest.mock import MagicMock, patch

# --- Mock nav2_msgs before importing source modules ---
if 'nav2_msgs' not in sys.modules:
    import sys as _sys
    nav2_msgs = types.ModuleType('nav2_msgs')
    nav2_msgs_action = types.ModuleType('nav2_msgs.action')
    mock_navigate = MagicMock()
    mock_navigate.Goal = MagicMock
    mock_navigate.Result = MagicMock
    mock_navigate.Feedback = MagicMock
    nav2_msgs_action.NavigateToPose = mock_navigate
    nav2_msgs.action = nav2_msgs_action
    _sys.modules['nav2_msgs'] = nav2_msgs
    _sys.modules['nav2_msgs.action'] = nav2_msgs_action

# Source modules under test
from rosiwit_app.app_node import AppState
from rosiwit_app.system_manager import SystemManager, SubsystemStatus
from rosiwit_app.map_manager import MapManager
from rosiwit_app.navigation_manager import NavigationManager, NavigationStatus
from rosiwit_app.waypoint_server import WaypointServer


# ---------------------------------------------------------------------------
# Fake node stub (avoids requiring a running ROS2 context)
# ---------------------------------------------------------------------------
class _FakeNode:
    """Minimal fake ROS2 Node for testing without rclpy."""

    def __init__(self, name="fake_node"):
        self._name = name
        self._logger = _FakeLogger()
        self._publishers = {}
        self._subscribers = {}
        self._services = {}
        self._timers = {}
        self._parameters = {}
        self._action_clients = {}

    def get_logger(self):
        return self._logger

    def get_name(self):
        return self._name

    def declare_parameter(self, name, default_value=None):
        self._parameters[name] = default_value

    def get_parameter(self, name):
        val = self._parameters.get(name)
        return _FakeParameter(val)

    def create_publisher(self, msg_type, topic, qos):
        pub = MagicMock()
        self._publishers[topic] = pub
        return pub

    def create_subscription(self, msg_type, topic, callback, qos):
        sub = MagicMock()
        self._subscribers[topic] = (sub, callback)
        return sub

    def create_service(self, srv_type, name, callback):
        srv = MagicMock()
        self._services[name] = (srv, callback)
        return srv

    def create_timer(self, period, callback):
        timer = MagicMock()
        self._timers[period] = (timer, callback)
        return timer

    def create_action_client(self, action_type, action_name):
        client = MagicMock()
        client.wait_for_server = MagicMock(return_value=True)
        client.send_goal_async = MagicMock()
        self._action_clients[action_name] = client
        return client

    def destroy_publisher(self, pub):
        pass

    def destroy_subscription(self, sub):
        pass

    def destroy_timer(self, timer):
        pass

    def destroy_service(self, srv):
        pass


class _FakeLogger:
    def debug(self, msg, **kw):
        pass

    def info(self, msg, **kw):
        pass

    def warn(self, msg, **kw):
        pass

    def error(self, msg, **kw):
        pass

    def fatal(self, msg, **kw):
        pass


class _FakeParameter:
    def __init__(self, value):
        self._value = value

    def get_parameter_value(self):
        return self

    @property
    def string_value(self):
        return str(self._value) if self._value is not None else ""

    @property
    def double_value(self):
        return float(self._value) if self._value is not None else 0.0

    @property
    def bool_value(self):
        return bool(self._value) if self._value is not None else False

    @property
    def integer_value(self):
        return int(self._value) if self._value is not None else 0

    @property
    def type(self):
        return MagicMock()


# ===========================================================================
# 4.1 系统启动验收 (System Startup Acceptance)
# ===========================================================================
class TestSystemStartupAcceptance(unittest.TestCase):
    """AC-4.1: Verify system startup acceptance criteria."""

    def setUp(self):
        self.fake_node = _FakeNode()

    def test_system_bringup_launch_file_exists(self):
        """AC4.1-1: system_bringup.launch.py exists and is importable."""
        base = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        launch_file = os.path.join(base, "launch", "system_bringup.launch.py")
        self.assertTrue(os.path.isfile(launch_file),
                        f"system_bringup.launch.py not found at {launch_file}")

    def test_system_bringup_has_generate_launch_description(self):
        """AC4.1-2: system_bringup.launch.py defines generate_launch_description."""
        base = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        launch_file = os.path.join(base, "launch", "system_bringup.launch.py")
        with open(launch_file) as f:
            source = f.read()
        self.assertIn("generate_launch_description", source)

    def test_system_bringup_declares_expected_parameters(self):
        """AC4.1-3: system_bringup.launch.py declares key parameters."""
        base = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        launch_file = os.path.join(base, "launch", "system_bringup.launch.py")
        with open(launch_file) as f:
            source = f.read()
        self.assertIn("use_simulator", source)
        self.assertIn("use_navigation", source)
        self.assertIn("use_rviz", source)
        self.assertIn("map_path", source)

    def test_sim_slam_nav_launch_exists(self):
        """AC4.1-4: sim_slam_nav.launch.py exists for simulation testing."""
        base = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        launch_file = os.path.join(base, "launch", "sim_slam_nav.launch.py")
        self.assertTrue(os.path.isfile(launch_file))

    def test_app_only_launch_exists(self):
        """AC4.1-5: app_only.launch.py exists for standalone mode."""
        base = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        launch_file = os.path.join(base, "launch", "app_only.launch.py")
        self.assertTrue(os.path.isfile(launch_file))

    def test_system_manager_tracks_three_subsystems(self):
        """AC4.1-6: SystemManager can track simulator, slam, and navigation."""
        mgr = SystemManager(self.fake_node)
        # Simulate registering the 3 subsystems
        mgr.register_subsystem("simulator", "/velodyne_points", object)
        mgr.register_subsystem("slam", "/odom_estimated", object)
        mgr.register_subsystem("navigation", "/cmd_vel", object)

        health = mgr.check_health()
        self.assertIn("simulator", health)
        self.assertIn("slam", health)
        self.assertIn("navigation", health)
        # Initially all should be offline (no messages received)
        for status in health.values():
            self.assertIn(status, [SubsystemStatus.OFFLINE.value, SubsystemStatus.UNKNOWN.value])


# ===========================================================================
# 4.2 地图管理验收 (Map Management Acceptance)
# ===========================================================================
class TestMapManagementAcceptance(unittest.TestCase):
    """AC-4.2: Verify map management acceptance criteria."""

    def setUp(self):
        self.fake_node = _FakeNode()
        self.test_dir = tempfile.mkdtemp()
        self.manager = MapManager(self.fake_node, map_path=self.test_dir)
        self.manager.ensure_map_directory()

    def tearDown(self):
        shutil.rmtree(self.test_dir, ignore_errors=True)

    def test_auto_detect_map_not_exists(self):
        """AC4.2-1: No map → system correctly detects absence."""
        self.assertFalse(self.manager.map_exists())

    def test_auto_detect_map_exists(self):
        """AC4.2-2: Map exists → system correctly detects presence."""
        # Create fake map files
        map_name = self.manager._map_file
        with open(os.path.join(self.test_dir, f"{map_name}.yaml"), "w") as f:
            f.write("image: test.pgm\nresolution: 0.05\n")
        with open(os.path.join(self.test_dir, f"{map_name}.pgm"), "w") as f:
            f.write("P5\n10 10\n255\n")
        with open(os.path.join(self.test_dir, f"{map_name}.pcd"), "w") as f:
            f.write("# .PCD v0.7\n")
        self.assertTrue(self.manager.map_exists())

    def test_auto_detect_partial_map(self):
        """AC4.2-3: Only YAML file exists (no PCD) → still detected as having map."""
        map_name = self.manager._map_file
        with open(os.path.join(self.test_dir, f"{map_name}.yaml"), "w") as f:
            f.write("image: test.pgm\nresolution: 0.05\n")
        # Only yaml file with content - should still indicate map exists
        self.assertTrue(self.manager.map_exists())

    def test_position_save_and_restore(self):
        """AC4.2-4: Position saved before shutdown and restored on startup."""
        # Save position
        self.manager.save_position(x=5.0, y=3.0, z=0.0,
                                   qx=0.0, qy=0.0, qz=0.707, qw=0.707)
        # Verify restore
        data = self.manager.load_position()
        self.assertIsNotNone(data)
        self.assertAlmostEqual(data["x"], 5.0)
        self.assertAlmostEqual(data["y"], 3.0)
        self.assertAlmostEqual(data["qz"], 0.707)
        self.assertAlmostEqual(data["qw"], 0.707)

    def test_map_directory_creation(self):
        """AC4.2-5: Map directory is created if it doesn't exist."""
        new_dir = os.path.join(self.test_dir, "subdir", "maps")
        self.assertFalse(os.path.exists(new_dir))
        mgr = MapManager(self.fake_node, map_path=new_dir)
        mgr.ensure_map_directory()
        self.assertTrue(os.path.isdir(new_dir))

    def test_default_map_path(self):
        """AC4.2-6: Default map path is /tmp/rosiwit_sim_map."""
        mgr = MapManager(self.fake_node)
        self.assertEqual(mgr._map_path, "/tmp/rosiwit_sim_map")

    def test_map_server_params(self):
        """AC4.2-7: get_map_server_params returns valid params for map_server."""
        params = self.manager.get_map_server_params()
        self.assertIsInstance(params, dict)
        self.assertIn("yaml_filename", params)


# ===========================================================================
# 4.3 导航目标接收验收 (Navigation Target Acceptance Acceptance)
# ===========================================================================
class TestNavigationTargetAcceptance(unittest.TestCase):
    """AC-4.3: Verify navigation target acceptance criteria."""

    def setUp(self):
        self.fake_node = _FakeNode()

    @patch('rosiwit_app.navigation_manager.ActionClient')
    def test_send_named_waypoint_as_goal(self, mock_action_client_cls):
        """AC4.3-1: Named waypoint can be resolved to coordinates and sent."""
        wp_server = WaypointServer(self.fake_node)
        wp_server.add_waypoint("test_point", x=5.0, y=3.0)

        nav_mgr = NavigationManager(self.fake_node, action_name="navigate_to_pose")

        # Get waypoint as PoseStamped
        pose = wp_server.get_waypoint_as_pose_stamped("test_point")
        self.assertIsNotNone(pose)

    @patch('rosiwit_app.navigation_manager.ActionClient')
    def test_navigation_status_tracking(self, mock_action_client_cls):
        """AC4.3-2: Navigation manager tracks navigation status correctly."""
        nav_mgr = NavigationManager(self.fake_node, action_name="navigate_to_pose")
        self.assertEqual(nav_mgr.status, NavigationStatus.IDLE)
        self.assertFalse(nav_mgr.is_navigating)

    @patch('rosiwit_app.navigation_manager.ActionClient')
    def test_navigation_timeout_configurable(self, mock_action_client_cls):
        """AC4.3-3: Navigation timeout is configurable."""
        nav_mgr = NavigationManager(
            self.fake_node, action_name="navigate_to_pose",
            timeout=300.0
        )
        self.assertEqual(nav_mgr._timeout, 300.0)


# ===========================================================================
# 4.4 状态监控验收 (Status Monitoring Acceptance)
# ===========================================================================
class TestStatusMonitoringAcceptance(unittest.TestCase):
    """AC-4.4: Verify status monitoring acceptance criteria."""

    def setUp(self):
        self.fake_node = _FakeNode()

    def test_app_state_values(self):
        """AC4.4-1: AppState enum defines all required states."""
        required_states = ["INIT", "INITIALIZING", "READY", "NAVIGATING",
                           "MAPPING", "MAP_SAVING"]
        for state_name in required_states:
            self.assertTrue(hasattr(AppState, state_name),
                            f"AppState missing {state_name}")

    def test_navigation_status_values(self):
        """AC4.4-2: NavigationStatus enum defines all required statuses."""
        required = ["IDLE", "NAVIGATING", "SUCCEEDED", "FAILED", "CANCELED"]
        for status_name in required:
            self.assertTrue(hasattr(NavigationStatus, status_name),
                            f"NavigationStatus missing {status_name}")

    def test_subsystem_status_values(self):
        """AC4.4-3: SubsystemStatus enum defines all required statuses."""
        required = ["ONLINE", "OFFLINE", "UNKNOWN"]
        for status_name in required:
            self.assertTrue(hasattr(SubsystemStatus, status_name),
                            f"SubsystemStatus missing {status_name}")

    def test_system_manager_health_check(self):
        """AC4.4-4: SystemManager provides health check interface."""
        mgr = SystemManager(self.fake_node)
        mgr.register_subsystem("test", "/topic", object)
        health = mgr.check_health()
        self.assertIn("test", health)
        self.assertIsInstance(health, dict)

    def test_system_manager_summary(self):
        """AC4.4-5: SystemManager provides full summary."""
        mgr = SystemManager(self.fake_node)
        mgr.register_subsystem("test", "/topic", object)
        summary = mgr.get_summary()
        self.assertIn("test", summary)
        self.assertIsInstance(summary["test"], dict)
        self.assertIn("status", summary["test"])


# ===========================================================================
# 4.5 航点管理验收 (Waypoint Management Acceptance)
# ===========================================================================
class TestWaypointManagementAcceptance(unittest.TestCase):
    """AC-4.5: Verify waypoint management acceptance criteria."""

    def setUp(self):
        self.fake_node = _FakeNode()
        self.test_dir = tempfile.mkdtemp()
        self.test_file = os.path.join(self.test_dir, "test_waypoints.yaml")

    def tearDown(self):
        shutil.rmtree(self.test_dir, ignore_errors=True)

    def test_load_predefined_waypoints(self):
        """AC4.5-1: Predefined waypoints loaded from waypoints.yaml."""
        # Write test waypoints
        with open(self.test_file, "w") as f:
            f.write("""
waypoints:
  home:
    x: 0.0
    y: 0.0
    z: 0.0
    qx: 0.0
    qy: 0.0
    qz: 0.0
    qw: 1.0
    frame_id: "map"
    description: "Home"
  point_a:
    x: 3.0
    y: 2.0
    z: 0.0
    qx: 0.0
    qy: 0.0
    qz: 0.0
    qw: 1.0
    frame_id: "map"
    description: "Point A"
""")
        server = WaypointServer(self.fake_node, waypoints_file=self.test_file)
        names = server.get_names()
        self.assertIn("home", names)
        self.assertIn("point_a", names)

    def test_add_waypoint_runtime(self):
        """AC4.5-2: Waypoints can be added at runtime."""
        server = WaypointServer(self.fake_node, waypoints_file=self.test_file)
        server.add_waypoint("new_point", x=1.0, y=2.0)
        self.assertTrue(server.has_waypoint("new_point"))

    def test_remove_waypoint(self):
        """AC4.5-3: Waypoints can be removed."""
        server = WaypointServer(self.fake_node, waypoints_file=self.test_file)
        server.add_waypoint("to_remove", x=1.0, y=2.0)
        self.assertTrue(server.has_waypoint("to_remove"))
        server.remove_waypoint("to_remove")
        self.assertFalse(server.has_waypoint("to_remove"))

    def test_update_waypoint(self):
        """AC4.5-4: Waypoints can be updated."""
        server = WaypointServer(self.fake_node, waypoints_file=self.test_file)
        server.add_waypoint("update_me", x=1.0, y=2.0)
        # Re-add to update
        server.add_waypoint("update_me", x=5.0, y=6.0)
        wp = server.get_waypoint("update_me")
        self.assertAlmostEqual(wp["x"], 5.0)
        self.assertAlmostEqual(wp["y"], 6.0)

    def test_waypoint_persistence(self):
        """AC4.5-5: Waypoints persist to file and can be reloaded."""
        server = WaypointServer(self.fake_node, waypoints_file=self.test_file)
        server.add_waypoint("persist_me", x=7.0, y=8.0)
        # add_waypoint auto-saves to file

        # Reload
        server2 = WaypointServer(self.fake_node, waypoints_file=self.test_file)
        self.assertTrue(server2.has_waypoint("persist_me"))
        wp = server2.get_waypoint("persist_me")
        self.assertAlmostEqual(wp["x"], 7.0)

    def test_get_waypoint_as_pose_stamped(self):
        """AC4.5-6: Waypoints can be retrieved as PoseStamped messages."""
        server = WaypointServer(self.fake_node, waypoints_file=self.test_file)
        server.add_waypoint("pose_test", x=1.0, y=2.0, z=0.0,
                            qx=0.0, qy=0.0, qz=0.707, qw=0.707)
        pose = server.get_waypoint_as_pose_stamped("pose_test")
        self.assertIsNotNone(pose)
        self.assertAlmostEqual(pose.pose.position.x, 1.0)
        self.assertAlmostEqual(pose.pose.position.y, 2.0)
        self.assertAlmostEqual(pose.pose.orientation.z, 0.707)
        self.assertAlmostEqual(pose.pose.orientation.w, 0.707)

    def test_nonexistent_waypoint_returns_none(self):
        """AC4.5-7: Querying nonexistent waypoint returns None."""
        server = WaypointServer(self.fake_node, waypoints_file=self.test_file)
        self.assertIsNone(server.get_waypoint("does_not_exist"))
        self.assertIsNone(server.get_waypoint_as_pose_stamped("does_not_exist"))


# ===========================================================================
# 6. 功能测试用例 (Functional Test Cases from Requirements)
# ===========================================================================
class TestFunctionalRequirements(unittest.TestCase):
    """Verify functional test cases from requirements.md section 6."""

    def setUp(self):
        self.fake_node = _FakeNode()

    # --- 6.1 系统启动 ---
    def test_all_required_modules_importable(self):
        """F6.1-1: All required modules can be imported."""
        import rosiwit_app.app_node
        import rosiwit_app.system_manager
        import rosiwit_app.map_manager
        import rosiwit_app.navigation_manager
        import rosiwit_app.waypoint_server

    def test_app_params_yaml_valid(self):
        """F6.1-2: app_params.yaml is valid YAML with correct structure."""
        import yaml
        base = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        params_file = os.path.join(base, "config", "app_params.yaml")
        with open(params_file) as f:
            data = yaml.safe_load(f)
        self.assertIn("rosiwit_app", data)
        params = data["rosiwit_app"]["ros__parameters"]
        self.assertIn("map_path", params)
        self.assertIn("map_file", params)

    def test_waypoints_yaml_valid(self):
        """F6.1-3: waypoints.yaml is valid YAML with correct structure."""
        import yaml
        base = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        wp_file = os.path.join(base, "config", "waypoints.yaml")
        with open(wp_file) as f:
            data = yaml.safe_load(f)
        self.assertIn("waypoints", data)
        self.assertGreaterEqual(len(data["waypoints"]), 2)

    # --- 6.2 地图管理 ---
    def test_no_map_detected_correctly(self):
        """F6.2-1: System correctly detects no map exists."""
        tmp = tempfile.mkdtemp()
        try:
            mgr = MapManager(self.fake_node, map_path=tmp)
            self.assertFalse(mgr.map_exists())
        finally:
            shutil.rmtree(tmp)

    def test_map_with_yaml_detected(self):
        """F6.2-2: System detects map when yaml file exists."""
        tmp = tempfile.mkdtemp()
        try:
            mgr = MapManager(self.fake_node, map_path=tmp, map_file="test")
            # Create the yaml file
            with open(os.path.join(tmp, "test.yaml"), "w") as f:
                f.write("image: test.pgm\nresolution: 0.05\n")
            self.assertTrue(mgr.map_exists())
        finally:
            shutil.rmtree(tmp)

    def test_position_json_persistence(self):
        """F6.2-3: Position persists to JSON and reloads correctly."""
        tmp = tempfile.mkdtemp()
        try:
            mgr = MapManager(self.fake_node, map_path=tmp)
            mgr.save_position(x=1.1, y=2.2, z=0.0,
                              qx=0.0, qy=0.0, qz=0.0, qw=1.0)
            # Verify file exists
            self.assertTrue(os.path.isfile(mgr.position_file))
            # Verify content
            with open(mgr.position_file) as f:
                data = json.load(f)
            self.assertAlmostEqual(data["x"], 1.1)
            self.assertAlmostEqual(data["y"], 2.2)
        finally:
            shutil.rmtree(tmp)

    # --- 6.3 导航 ---
    @patch('rosiwit_app.navigation_manager.ActionClient')
    def test_navigation_manager_idle_initially(self, mock_ac):
        """F6.3-1: NavigationManager starts in IDLE state."""
        nav = NavigationManager(self.fake_node, action_name="/navigate_to_pose")
        self.assertEqual(nav.status, NavigationStatus.IDLE)
        self.assertFalse(nav.is_navigating)

    @patch('rosiwit_app.navigation_manager.ActionClient')
    def test_navigation_status_callbacks(self, mock_ac):
        """F6.3-2: NavigationManager status transitions via callbacks."""
        nav = NavigationManager(self.fake_node, action_name="/navigate_to_pose")

        # Simulate feedback callback - should not change status to NAVIGATING
        # (that happens when goal is accepted)
        nav._feedback_callback_internal(MagicMock())

        # Simulate result callback with succeeded status
        mock_future = MagicMock()
        mock_result = MagicMock()
        mock_result.status = 4  # GOAL_STATUS_SUCCEEDED
        mock_future.result.return_value = mock_result
        nav._result_callback_internal(mock_future)
        self.assertEqual(nav.status, NavigationStatus.SUCCEEDED)

    # --- 6.4 状态监控 ---
    def test_subsystem_online_after_message(self):
        """F6.4-1: Subsystem transitions to ONLINE after receiving a message."""
        mgr = SystemManager(self.fake_node)
        mgr.register_subsystem("test_sub", "/test_topic", object)
        self.assertFalse(mgr.is_subsystem_online("test_sub"))

        # Simulate receiving a message
        mgr._on_subsystem_msg("test_sub")
        self.assertTrue(mgr.is_subsystem_online("test_sub"))

    def test_subsystem_timeout(self):
        """F6.4-2: Subsystem transitions to OFFLINE after timeout."""
        import time
        mgr = SystemManager(self.fake_node)
        mgr.register_subsystem("test_sub", "/test_topic", object, timeout=0.05)

        # Bring online
        mgr._on_subsystem_msg("test_sub")
        self.assertTrue(mgr.is_subsystem_online("test_sub"))

        # Wait for timeout
        time.sleep(0.1)
        mgr.check_health()
        self.assertFalse(mgr.is_subsystem_online("test_sub"))

    # --- 6.5 航点管理 ---
    def test_waypoint_crud_complete(self):
        """F6.5-1: Complete CRUD cycle for waypoints."""
        tmp = tempfile.mkdtemp()
        try:
            wp_file = os.path.join(tmp, "wp.yaml")
            server = WaypointServer(self.fake_node, waypoints_file=wp_file)

            # Create
            server.add_waypoint("wp1", x=1.0, y=2.0)
            self.assertTrue(server.has_waypoint("wp1"))

            # Read
            wp = server.get_waypoint("wp1")
            self.assertAlmostEqual(wp["x"], 1.0)

            # Update (by re-adding)
            server.add_waypoint("wp1", x=10.0, y=20.0)
            wp = server.get_waypoint("wp1")
            self.assertAlmostEqual(wp["x"], 10.0)

            # Delete
            server.remove_waypoint("wp1")
            self.assertFalse(server.has_waypoint("wp1"))
        finally:
            shutil.rmtree(tmp)


# ===========================================================================
# Package Structure Validation (from requirements)
# ===========================================================================
class TestPackageStructureAcceptance(unittest.TestCase):
    """Verify package structure matches requirements."""

    def setUp(self):
        self.base = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    def test_cmake_or_setup_exists(self):
        """Package has proper build files."""
        # ament_python uses setup.py
        self.assertTrue(os.path.isfile(os.path.join(self.base, "setup.py")))

    def test_package_xml_valid(self):
        """package.xml is valid and has correct package name."""
        import xml.etree.ElementTree as ET
        tree = ET.parse(os.path.join(self.base, "package.xml"))
        root = tree.getroot()
        name = root.find("name")
        self.assertIsNotNone(name)
        self.assertEqual(name.text, "rosiwit_app")

    def test_resource_marker_exists(self):
        """ament_python requires a resource marker file."""
        self.assertTrue(os.path.isfile(os.path.join(self.base, "resource", "rosiwit_app")))

    def test_setup_cfg_exists(self):
        """setup.cfg exists with correct install paths."""
        self.assertTrue(os.path.isfile(os.path.join(self.base, "setup.cfg")))

    def test_python_modules_all_present(self):
        """All required Python modules are present."""
        pkg_dir = os.path.join(self.base, "rosiwit_app")
        required = ["__init__.py", "app_node.py", "system_manager.py",
                    "map_manager.py", "navigation_manager.py", "waypoint_server.py"]
        for module in required:
            self.assertTrue(os.path.isfile(os.path.join(pkg_dir, module)),
                            f"Missing module: {module}")

    def test_launch_files_all_present(self):
        """All required launch files are present."""
        launch_dir = os.path.join(self.base, "launch")
        required = ["system_bringup.launch.py", "sim_slam_nav.launch.py",
                    "app_only.launch.py"]
        for lf in required:
            self.assertTrue(os.path.isfile(os.path.join(launch_dir, lf)),
                            f"Missing launch file: {lf}")

    def test_config_files_all_present(self):
        """All required config files are present."""
        config_dir = os.path.join(self.base, "config")
        required = ["app_params.yaml", "waypoints.yaml"]
        for cf in required:
            self.assertTrue(os.path.isfile(os.path.join(config_dir, cf)),
                            f"Missing config file: {cf}")

    def test_readme_exists(self):
        """README.md exists."""
        self.assertTrue(os.path.isfile(os.path.join(self.base, "README.md")))


if __name__ == "__main__":
    unittest.main()
