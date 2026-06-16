"""Unit tests for MapManager.

Tests map file detection, position persistence (save/load),
directory management, and path generation.
"""

import json
import os
import shutil
import tempfile
import unittest
from unittest.mock import MagicMock

from rosiwit_app.map_manager import MapManager


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


class TestMapManagerPaths(unittest.TestCase):
    """Test path generation properties."""

    def setUp(self):
        self.fake_node = _FakeNode()
        self.manager = MapManager(self.fake_node, map_path="/tmp/test_map", map_file="test_map")

    def test_map_path_property(self):
        """map_path returns configured directory."""
        self.assertEqual(self.manager.map_path, "/tmp/test_map")

    def test_map_file_property(self):
        """map_file returns configured base name."""
        self.assertEqual(self.manager.map_file, "test_map")

    def test_map_yaml_path(self):
        """map_yaml_path joins map_path with .yaml extension."""
        self.assertEqual(self.manager.map_yaml_path, "/tmp/test_map/test_map.yaml")

    def test_map_pgm_path(self):
        """map_pgm_path joins map_path with .pgm extension."""
        self.assertEqual(self.manager.map_pgm_path, "/tmp/test_map/test_map.pgm")

    def test_map_pcd_path(self):
        """map_pcd_path joins map_path with .pcd extension."""
        self.assertEqual(self.manager.map_pcd_path, "/tmp/test_map/test_map.pcd")

    def test_position_file_path(self):
        """position_file points to last_position.json in map_path."""
        self.assertEqual(self.manager.position_file, "/tmp/test_map/last_position.json")


class TestMapManagerDirectory(unittest.TestCase):
    """Test directory creation and map existence detection."""

    def setUp(self):
        self.test_dir = tempfile.mkdtemp()
        self.fake_node = _FakeNode()
        self.manager = MapManager(self.fake_node, map_path=self.test_dir, map_file="fast_lio2_map")

    def tearDown(self):
        shutil.rmtree(self.test_dir, ignore_errors=True)

    def test_ensure_map_directory_creates_dir(self):
        """ensure_map_directory creates the directory if missing."""
        new_dir = os.path.join(self.test_dir, "new_subdir")
        mgr = MapManager(self.fake_node, map_path=new_dir)
        self.assertFalse(os.path.isdir(new_dir))
        result = mgr.ensure_map_directory()
        self.assertTrue(result)
        self.assertTrue(os.path.isdir(new_dir))

    def test_ensure_map_directory_existing(self):
        """ensure_map_directory succeeds on existing directory."""
        result = self.manager.ensure_map_directory()
        self.assertTrue(result)

    def test_map_exists_false_when_no_files(self):
        """map_exists returns False when no map files exist."""
        self.assertFalse(self.manager.map_exists())

    def test_map_exists_true_when_yaml_and_pgm(self):
        """map_exists returns True when .yaml and .pgm files exist."""
        # Create both required files
        open(os.path.join(self.test_dir, "fast_lio2_map.yaml"), "w").close()
        open(os.path.join(self.test_dir, "fast_lio2_map.pgm"), "w").close()
        self.assertTrue(self.manager.map_exists())

    def test_map_exists_false_when_only_yaml(self):
        """map_exists returns False if only .yaml exists."""
        open(os.path.join(self.test_dir, "fast_lio2_map.yaml"), "w").close()
        self.assertFalse(self.manager.map_exists())

    def test_pcd_exists_false(self):
        """pcd_exists returns False when no .pcd file."""
        self.assertFalse(self.manager.pcd_exists())

    def test_pcd_exists_true(self):
        """pcd_exists returns True when .pcd file exists."""
        open(os.path.join(self.test_dir, "fast_lio2_map.pcd"), "w").close()
        self.assertTrue(self.manager.pcd_exists())


class TestMapManagerPosition(unittest.TestCase):
    """Test position save and load persistence."""

    def setUp(self):
        self.test_dir = tempfile.mkdtemp()
        self.fake_node = _FakeNode()
        self.manager = MapManager(self.fake_node, map_path=self.test_dir)
        self.manager.ensure_map_directory()

    def tearDown(self):
        shutil.rmtree(self.test_dir, ignore_errors=True)

    def test_save_and_load_position(self):
        """save_position persists data that load_position recovers."""
        result = self.manager.save_position(x=1.0, y=2.0, z=0.0, qx=0.0, qy=0.0, qz=0.0, qw=1.0)
        self.assertTrue(result)
        data = self.manager.load_position()
        self.assertIsNotNone(data)
        self.assertAlmostEqual(data["x"], 1.0)
        self.assertAlmostEqual(data["y"], 2.0)
        self.assertAlmostEqual(data["z"], 0.0)
        self.assertAlmostEqual(data["qx"], 0.0)
        self.assertAlmostEqual(data["qy"], 0.0)
        self.assertAlmostEqual(data["qz"], 0.0)
        self.assertAlmostEqual(data["qw"], 1.0)

    def test_load_position_returns_none_when_no_file(self):
        """load_position returns None if file doesn't exist."""
        data = self.manager.load_position()
        self.assertIsNone(data)

    def test_save_position_overwrite(self):
        """Second save_position overwrites the first."""
        self.manager.save_position(x=1.0, y=2.0, z=0.0, qx=0.0, qy=0.0, qz=0.0, qw=1.0)
        self.manager.save_position(x=3.0, y=4.0, z=0.5, qx=0.1, qy=0.2, qz=0.3, qw=0.9)
        data = self.manager.load_position()
        self.assertAlmostEqual(data["x"], 3.0)
        self.assertAlmostEqual(data["y"], 4.0)

    def test_save_position_negative_coords(self):
        """save_position handles negative coordinates."""
        self.manager.save_position(x=-5.5, y=-3.2, z=-0.1, qx=0.0, qy=0.0, qz=0.707, qw=0.707)
        data = self.manager.load_position()
        self.assertAlmostEqual(data["x"], -5.5)
        self.assertAlmostEqual(data["y"], -3.2)

    def test_load_position_handles_corrupt_file(self):
        """load_position returns None for corrupt JSON."""
        with open(self.manager.position_file, "w") as f:
            f.write("not valid json {{{")
        data = self.manager.load_position()
        self.assertIsNone(data)

    def test_get_map_server_params(self):
        """get_map_server_params returns correct dict."""
        params = self.manager.get_map_server_params()
        self.assertIn("yaml_filename", params)
        self.assertEqual(params["yaml_filename"], self.manager.map_yaml_path)

    def test_get_map_info(self):
        """get_map_info returns map status information."""
        info = self.manager.get_map_info()
        self.assertIn("map_path", info)
        self.assertIn("map_exists", info)
        self.assertFalse(info["map_exists"])


if __name__ == "__main__":
    unittest.main()
