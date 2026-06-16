"""Unit tests for WaypointServer.

Tests waypoint CRUD operations, YAML persistence,
and PoseStamped conversion.
"""

import os
import shutil
import tempfile
import unittest
from unittest.mock import MagicMock

import yaml

from rosiwit_app.waypoint_server import WaypointServer


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


class TestWaypointServerNoFile(unittest.TestCase):
    """Test WaypointServer without a YAML file."""

    def setUp(self):
        self.fake_node = _FakeNode()
        self.server = WaypointServer(self.fake_node)

    def test_initial_empty(self):
        """New server without file has zero waypoints."""
        self.assertEqual(len(self.server.list_waypoints()), 0)
        self.assertEqual(len(self.server.get_names()), 0)

    def test_has_waypoint_false(self):
        """has_waypoint returns False for non-existent waypoint."""
        self.assertFalse(self.server.has_waypoint("dock"))

    def test_get_waypoint_none(self):
        """get_waypoint returns None for non-existent waypoint."""
        self.assertIsNone(self.server.get_waypoint("nonexistent"))


class TestWaypointServerCRUD(unittest.TestCase):
    """Test add, get, remove, list operations."""

    def setUp(self):
        self.test_dir = tempfile.mkdtemp()
        self.wp_file = os.path.join(self.test_dir, "waypoints.yaml")
        self.fake_node = _FakeNode()
        self.server = WaypointServer(self.fake_node, waypoints_file=self.wp_file)

    def tearDown(self):
        shutil.rmtree(self.test_dir, ignore_errors=True)

    def test_add_waypoint(self):
        """add_waypoint creates a new waypoint and persists."""
        result = self.server.add_waypoint("dock", 1.0, 2.0)
        self.assertTrue(result)
        self.assertTrue(self.server.has_waypoint("dock"))

    def test_add_waypoint_data(self):
        """Added waypoint has correct coordinate data."""
        self.server.add_waypoint("dock", 1.5, 2.5, 0.0, 0.0, 0.0, 0.0, 1.0,
                                 frame_id="map", description="Docking station")
        wp = self.server.get_waypoint("dock")
        self.assertAlmostEqual(wp["x"], 1.5)
        self.assertAlmostEqual(wp["y"], 2.5)
        self.assertEqual(wp["frame_id"], "map")
        self.assertEqual(wp["description"], "Docking station")

    def test_add_waypoint_defaults(self):
        """add_waypoint defaults z=0, q=(0,0,0,1), frame='map'."""
        self.server.add_waypoint("home", 0.0, 0.0)
        wp = self.server.get_waypoint("home")
        self.assertAlmostEqual(wp["z"], 0.0)
        self.assertAlmostEqual(wp["qw"], 1.0)
        self.assertEqual(wp["frame_id"], "map")

    def test_add_multiple_waypoints(self):
        """Multiple waypoints can be added."""
        self.server.add_waypoint("a", 1.0, 0.0)
        self.server.add_waypoint("b", 2.0, 0.0)
        self.server.add_waypoint("c", 3.0, 0.0)
        names = self.server.get_names()
        self.assertEqual(len(names), 3)
        self.assertIn("a", names)
        self.assertIn("b", names)
        self.assertIn("c", names)

    def test_update_waypoint(self):
        """add_waypoint with same name overwrites."""
        self.server.add_waypoint("dock", 1.0, 2.0)
        self.server.add_waypoint("dock", 3.0, 4.0)
        wp = self.server.get_waypoint("dock")
        self.assertAlmostEqual(wp["x"], 3.0)
        self.assertAlmostEqual(wp["y"], 4.0)

    def test_remove_waypoint(self):
        """remove_waypoint deletes the waypoint."""
        self.server.add_waypoint("dock", 1.0, 2.0)
        self.assertTrue(self.server.remove_waypoint("dock"))
        self.assertFalse(self.server.has_waypoint("dock"))

    def test_remove_nonexistent_waypoint(self):
        """remove_waypoint returns False for non-existent waypoint."""
        self.assertFalse(self.server.remove_waypoint("ghost"))

    def test_list_waypoints_returns_copy(self):
        """list_waypoints returns a copy, not the internal dict."""
        self.server.add_waypoint("a", 1.0, 2.0)
        listed = self.server.list_waypoints()
        listed["a"]["x"] = 999.0  # mutate the copy
        # Original should be unaffected
        self.assertAlmostEqual(self.server.get_waypoint("a")["x"], 1.0)

    def test_get_names_returns_list(self):
        """get_names returns a list of strings."""
        self.server.add_waypoint("alpha", 0.0, 0.0)
        self.server.add_waypoint("beta", 1.0, 1.0)
        names = self.server.get_names()
        self.assertIsInstance(names, list)
        self.assertEqual(len(names), 2)

    def test_get_summary(self):
        """get_summary returns count, names, and file path."""
        self.server.add_waypoint("dock", 1.0, 2.0)
        summary = self.server.get_summary()
        self.assertEqual(summary["count"], 1)
        self.assertIn("dock", summary["names"])
        self.assertEqual(summary["file"], self.wp_file)


class TestWaypointServerPersistence(unittest.TestCase):
    """Test YAML file persistence."""

    def setUp(self):
        self.test_dir = tempfile.mkdtemp()
        self.wp_file = os.path.join(self.test_dir, "waypoints.yaml")
        self.fake_node = _FakeNode()

    def tearDown(self):
        shutil.rmtree(self.test_dir, ignore_errors=True)

    def test_save_creates_yaml_file(self):
        """add_waypoint creates the YAML file."""
        server = WaypointServer(self.fake_node, waypoints_file=self.wp_file)
        server.add_waypoint("dock", 1.0, 2.0)
        self.assertTrue(os.path.isfile(self.wp_file))

    def test_yaml_file_has_waypoints_key(self):
        """YAML file has 'waypoints' top-level key."""
        server = WaypointServer(self.fake_node, waypoints_file=self.wp_file)
        server.add_waypoint("dock", 1.0, 2.0)
        with open(self.wp_file) as f:
            data = yaml.safe_load(f)
        self.assertIn("waypoints", data)
        self.assertIn("dock", data["waypoints"])

    def test_reload_from_file(self):
        """WaypointServer loads waypoints from existing YAML file."""
        # First server creates and saves
        server1 = WaypointServer(self.fake_node, waypoints_file=self.wp_file)
        server1.add_waypoint("dock", 1.0, 2.0)
        server1.add_waypoint("home", 3.0, 4.0)

        # Second server loads from same file
        server2 = WaypointServer(self.fake_node, waypoints_file=self.wp_file)
        self.assertEqual(len(server2.get_names()), 2)
        self.assertTrue(server2.has_waypoint("dock"))
        self.assertTrue(server2.has_waypoint("home"))

    def test_remove_persists_to_file(self):
        """remove_waypoint persists the removal to file."""
        server = WaypointServer(self.fake_node, waypoints_file=self.wp_file)
        server.add_waypoint("dock", 1.0, 2.0)
        server.add_waypoint("home", 3.0, 4.0)
        server.remove_waypoint("dock")

        # Reload
        server2 = WaypointServer(self.fake_node, waypoints_file=self.wp_file)
        self.assertFalse(server2.has_waypoint("dock"))
        self.assertTrue(server2.has_waypoint("home"))


class TestWaypointServerPoseStamped(unittest.TestCase):
    """Test get_waypoint_as_pose_stamped conversion."""

    def setUp(self):
        self.fake_node = _FakeNode()
        self.server = WaypointServer(self.fake_node)
        self.server.add_waypoint("dock", 1.5, 2.5, 0.1, 0.0, 0.0, 0.707, 0.707)

    def test_returns_pose_stamped(self):
        """get_waypoint_as_pose_stamped returns a PoseStamped object."""
        from geometry_msgs.msg import PoseStamped
        pose = self.server.get_waypoint_as_pose_stamped("dock")
        self.assertIsInstance(pose, PoseStamped)

    def test_position_correct(self):
        """Converted PoseStamped has correct position."""
        pose = self.server.get_waypoint_as_pose_stamped("dock")
        self.assertAlmostEqual(pose.pose.position.x, 1.5)
        self.assertAlmostEqual(pose.pose.position.y, 2.5)
        self.assertAlmostEqual(pose.pose.position.z, 0.1)

    def test_orientation_correct(self):
        """Converted PoseStamped has correct orientation."""
        pose = self.server.get_waypoint_as_pose_stamped("dock")
        self.assertAlmostEqual(pose.pose.orientation.x, 0.0)
        self.assertAlmostEqual(pose.pose.orientation.y, 0.0)
        self.assertAlmostEqual(pose.pose.orientation.z, 0.707)
        self.assertAlmostEqual(pose.pose.orientation.w, 0.707)

    def test_frame_id_correct(self):
        """Converted PoseStamped has correct frame_id."""
        pose = self.server.get_waypoint_as_pose_stamped("dock")
        self.assertEqual(pose.header.frame_id, "map")

    def test_nonexistent_returns_none(self):
        """get_waypoint_as_pose_stamped returns None for missing waypoint."""
        result = self.server.get_waypoint_as_pose_stamped("ghost")
        self.assertIsNone(result)


if __name__ == "__main__":
    unittest.main()
