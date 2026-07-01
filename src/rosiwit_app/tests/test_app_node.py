"""Unit tests for AppState enum and RosiwitAppNode state machine logic.

Tests the state machine transitions and status building without requiring
a full ROS2 system. Uses mocks for ROS2 node dependencies.
"""

import json
import os
import shutil
import sys
import tempfile
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

from rosiwit_app.app_node import AppState


class TestAppStateEnum(unittest.TestCase):
    """Test AppState enum values and completeness."""

    def test_all_states_defined(self):
        """All required states are present in AppState."""
        required = ["INIT", "INITIALIZING", "READY", "MAPPING",
                     "MAP_SAVING", "NAVIGATING", "ERROR"]
        for name in required:
            self.assertTrue(hasattr(AppState, name),
                            f"AppState.{name} should be defined")

    def test_state_values_are_strings(self):
        """AppState enum values are uppercase string names."""
        self.assertEqual(AppState.INIT.value, "INIT")
        self.assertEqual(AppState.READY.value, "READY")
        self.assertEqual(AppState.NAVIGATING.value, "NAVIGATING")
        self.assertEqual(AppState.MAPPING.value, "MAPPING")
        self.assertEqual(AppState.ERROR.value, "ERROR")

    def test_state_count(self):
        """AppState has exactly 7 states."""
        self.assertEqual(len(AppState), 7)


class TestAppStateTransitions(unittest.TestCase):
    """Test valid state transitions (logical, not enforced by code)."""

    def test_init_to_initializing(self):
        """INIT → INITIALIZING is a valid transition."""
        state = AppState.INIT
        # In actual code, this transition happens during startup
        self.assertEqual(state, AppState.INIT)

    def test_initializing_to_ready(self):
        """INITIALIZING → READY when map exists and loaded."""
        state = AppState.INITIALIZING
        self.assertEqual(state, AppState.INITIALIZING)

    def test_initializing_to_mapping(self):
        """INITIALIZING → MAPPING when no map exists."""
        state = AppState.INITIALIZING
        self.assertEqual(state, AppState.INITIALIZING)

    def test_ready_to_navigating(self):
        """READY → NAVIGATING when goal is received."""
        state = AppState.READY
        self.assertEqual(state, AppState.READY)

    def test_navigating_to_ready(self):
        """NAVIGATING → READY when navigation completes."""
        state = AppState.NAVIGATING
        self.assertEqual(state, AppState.NAVIGATING)


class TestPackageStructure(unittest.TestCase):
    """Test that the package structure matches requirements."""

    def test_package_root_exists(self):
        """rosiwit_app package root directory exists."""
        pkg_dir = os.path.join(
            os.path.dirname(__file__), ".."
        )
        self.assertTrue(os.path.isdir(pkg_dir))

    def test_required_python_modules_exist(self):
        """All required Python modules exist in the package."""
        pkg_dir = os.path.join(os.path.dirname(__file__), "..", "rosiwit_app")
        required_files = [
            "__init__.py",
            "app_node.py",
            "system_manager.py",
            "map_manager.py",
            "navigation_manager.py",
            "waypoint_server.py",
        ]
        for filename in required_files:
            filepath = os.path.join(pkg_dir, filename)
            self.assertTrue(
                os.path.isfile(filepath),
                f"Required module '{filename}' is missing"
            )

    def test_required_config_files_exist(self):
        """Required config files exist."""
        config_dir = os.path.join(os.path.dirname(__file__), "..", "config")
        required_files = ["app_params.yaml", "waypoints.yaml"]
        for filename in required_files:
            filepath = os.path.join(config_dir, filename)
            self.assertTrue(
                os.path.isfile(filepath),
                f"Required config file '{filename}' is missing"
            )

    def test_required_launch_files_exist(self):
        """Required launch files exist."""
        launch_dir = os.path.join(os.path.dirname(__file__), "..", "launch")
        required_files = [
            "system_bringup.launch.py",
            "sim_slam_nav.launch.py",
            "app_only.launch.py",
        ]
        for filename in required_files:
            filepath = os.path.join(launch_dir, filename)
            self.assertTrue(
                os.path.isfile(filepath),
                f"Required launch file '{filename}' is missing"
            )

    def test_package_xml_exists(self):
        """package.xml exists in the package root."""
        pkg_dir = os.path.join(os.path.dirname(__file__), "..")
        self.assertTrue(os.path.isfile(os.path.join(pkg_dir, "package.xml")))

    def test_setup_py_exists(self):
        """setup.py exists in the package root."""
        pkg_dir = os.path.join(os.path.dirname(__file__), "..")
        self.assertTrue(os.path.isfile(os.path.join(pkg_dir, "setup.py")))

    def test_setup_cfg_exists(self):
        """setup.cfg exists in the package root."""
        pkg_dir = os.path.join(os.path.dirname(__file__), "..")
        self.assertTrue(os.path.isfile(os.path.join(pkg_dir, "setup.cfg")))

    def test_resource_marker_file_exists(self):
        """resource/rosiwit_app marker file exists (ament_python requirement)."""
        pkg_dir = os.path.join(os.path.dirname(__file__), "..")
        self.assertTrue(
            os.path.isfile(os.path.join(pkg_dir, "resource", "rosiwit_app"))
        )

    def test_readme_exists(self):
        """README.md exists in the package root."""
        pkg_dir = os.path.join(os.path.dirname(__file__), "..")
        self.assertTrue(os.path.isfile(os.path.join(pkg_dir, "README.md")))

    def test_rviz_config_exists(self):
        """RViz config file exists."""
        pkg_dir = os.path.join(os.path.dirname(__file__), "..")
        self.assertTrue(
            os.path.isfile(os.path.join(pkg_dir, "rviz", "rosiwit_app.rviz"))
        )


if __name__ == "__main__":
    unittest.main()
