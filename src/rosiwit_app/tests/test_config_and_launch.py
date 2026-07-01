"""Tests for configuration files and launch file structure.

Validates YAML syntax, required parameter presence, and launch file
correctness for the rosiwit_app package.
"""

import ast
import os
import unittest
import xml.etree.ElementTree as ET

import yaml


class TestAppParamsYaml(unittest.TestCase):
    """Test app_params.yaml configuration file."""

    def setUp(self):
        config_dir = os.path.join(os.path.dirname(__file__), "..", "config")
        self.config_path = os.path.join(config_dir, "app_params.yaml")
        with open(self.config_path, "r") as f:
            self.config = yaml.safe_load(f)

    def test_yaml_is_valid(self):
        """app_params.yaml parses as valid YAML."""
        self.assertIsNotNone(self.config)

    def test_top_level_key_is_rosiwit_app(self):
        """Top-level key is 'rosiwit_app'."""
        self.assertIn("rosiwit_app", self.config)

    def test_has_ros_parameters(self):
        """Config has ros__parameters section."""
        params = self.config["rosiwit_app"]
        self.assertIn("ros__parameters", params)

    def test_required_parameters_present(self):
        """All required parameters are present in the config."""
        params = self.config["rosiwit_app"]["ros__parameters"]
        required_keys = [
            "map_path",
            "map_file",
            "auto_save_position",
            "position_save_interval",
            "startup_timeout",
            "health_check_interval",
            "navigation_timeout",
            "goal_reached_tolerance",
            "auto_load_map",
            "auto_start_mapping",
        ]
        for key in required_keys:
            self.assertIn(key, params,
                          f"Required parameter '{key}' missing from app_params.yaml")

    def test_map_path_is_string(self):
        """map_path parameter is a string."""
        params = self.config["rosiwit_app"]["ros__parameters"]
        self.assertIsInstance(params["map_path"], str)

    def test_default_map_path(self):
        """map_path defaults to /tmp/rosiwit_sim_map."""
        params = self.config["rosiwit_app"]["ros__parameters"]
        self.assertEqual(params["map_path"], "/tmp/rosiwit_sim_map")

    def test_map_file_is_string(self):
        """map_file parameter is a string."""
        params = self.config["rosiwit_app"]["ros__parameters"]
        self.assertIsInstance(params["map_file"], str)

    def test_timeout_values_are_numeric(self):
        """Timeout parameters are numeric (int or float)."""
        params = self.config["rosiwit_app"]["ros__parameters"]
        timeout_keys = [
            "startup_timeout",
            "health_check_interval",
            "navigation_timeout",
            "position_save_interval",
        ]
        for key in timeout_keys:
            val = params[key]
            self.assertTrue(
                isinstance(val, (int, float)),
                f"'{key}' should be numeric, got {type(val).__name__}"
            )
            self.assertGreater(val, 0, f"'{key}' should be positive")

    def test_boolean_parameters(self):
        """Boolean parameters are bool type."""
        params = self.config["rosiwit_app"]["ros__parameters"]
        bool_keys = ["auto_save_position", "auto_load_map", "auto_start_mapping"]
        for key in bool_keys:
            self.assertIsInstance(params[key], bool,
                                  f"'{key}' should be bool, got {type(params[key]).__name__}")

    def test_goal_reached_tolerance_positive(self):
        """goal_reached_tolerance is a positive number."""
        params = self.config["rosiwit_app"]["ros__parameters"]
        self.assertGreater(params["goal_reached_tolerance"], 0)


class TestWaypointsYaml(unittest.TestCase):
    """Test waypoints.yaml configuration file."""

    def setUp(self):
        config_dir = os.path.join(os.path.dirname(__file__), "..", "config")
        self.config_path = os.path.join(config_dir, "waypoints.yaml")
        with open(self.config_path, "r") as f:
            self.config = yaml.safe_load(f)

    def test_yaml_is_valid(self):
        """waypoints.yaml parses as valid YAML."""
        self.assertIsNotNone(self.config)

    def test_has_waypoints_key(self):
        """Top-level 'waypoints' key exists."""
        self.assertIn("waypoints", self.config)

    def test_has_home_waypoint(self):
        """'home' waypoint exists."""
        self.assertIn("home", self.config["waypoints"])

    def test_home_is_at_origin(self):
        """'home' waypoint has (0,0,0) position."""
        home = self.config["waypoints"]["home"]
        self.assertAlmostEqual(home["x"], 0.0)
        self.assertAlmostEqual(home["y"], 0.0)
        self.assertAlmostEqual(home["z"], 0.0)

    def test_home_has_identity_rotation(self):
        """'home' waypoint has identity quaternion."""
        home = self.config["waypoints"]["home"]
        self.assertAlmostEqual(home["qx"], 0.0)
        self.assertAlmostEqual(home["qy"], 0.0)
        self.assertAlmostEqual(home["qz"], 0.0)
        self.assertAlmostEqual(home["qw"], 1.0)

    def test_each_waypoint_has_required_fields(self):
        """Every waypoint has x, y, z, qx, qy, qz, qw fields."""
        required_fields = ["x", "y", "z", "qx", "qy", "qz", "qw"]
        for name, wp in self.config["waypoints"].items():
            for field in required_fields:
                self.assertIn(field, wp,
                              f"Waypoint '{name}' missing field '{field}'")

    def test_quaternions_are_normalized_approximately(self):
        """Quaternion values are approximately normalized (||q|| ≈ 1)."""
        for name, wp in self.config["waypoints"].items():
            norm_sq = (wp["qx"]**2 + wp["qy"]**2 + wp["qz"]**2 + wp["qw"]**2)
            self.assertAlmostEqual(
                norm_sq, 1.0, places=2,
                msg=f"Waypoint '{name}' quaternion is not normalized: ||q||^2 = {norm_sq}"
            )

    def test_at_least_three_waypoints(self):
        """At least 3 waypoints are predefined (home, point_a, point_b)."""
        self.assertGreaterEqual(
            len(self.config["waypoints"]), 3,
            "Should have at least 3 predefined waypoints"
        )


class TestLaunchFilesSyntax(unittest.TestCase):
    """Test launch files have valid Python syntax."""

    def _get_launch_dir(self):
        return os.path.join(os.path.dirname(__file__), "..", "launch")

    def test_system_bringup_syntax(self):
        """system_bringup.launch.py has valid Python syntax."""
        path = os.path.join(self._get_launch_dir(), "system_bringup.launch.py")
        with open(path, "r") as f:
            source = f.read()
        ast.parse(source)

    def test_sim_slam_nav_syntax(self):
        """sim_slam_nav.launch.py has valid Python syntax."""
        path = os.path.join(self._get_launch_dir(), "sim_slam_nav.launch.py")
        with open(path, "r") as f:
            source = f.read()
        ast.parse(source)

    def test_app_only_syntax(self):
        """app_only.launch.py has valid Python syntax."""
        path = os.path.join(self._get_launch_dir(), "app_only.launch.py")
        with open(path, "r") as f:
            source = f.read()
        ast.parse(source)


class TestLaunchFilesStructure(unittest.TestCase):
    """Test launch files have required structure (generate_launch_description)."""

    def _get_launch_dir(self):
        return os.path.join(os.path.dirname(__file__), "..", "launch")

    def test_system_bringup_has_generate_function(self):
        """system_bringup.launch.py defines generate_launch_description."""
        path = os.path.join(self._get_launch_dir(), "system_bringup.launch.py")
        with open(path, "r") as f:
            source = f.read()
        self.assertIn("generate_launch_description", source)

    def test_sim_slam_nav_has_generate_function(self):
        """sim_slam_nav.launch.py defines generate_launch_description."""
        path = os.path.join(self._get_launch_dir(), "sim_slam_nav.launch.py")
        with open(path, "r") as f:
            source = f.read()
        self.assertIn("generate_launch_description", source)

    def test_app_only_has_generate_function(self):
        """app_only.launch.py defines generate_launch_description."""
        path = os.path.join(self._get_launch_dir(), "app_only.launch.py")
        with open(path, "r") as f:
            source = f.read()
        self.assertIn("generate_launch_description", source)


class TestPackageXml(unittest.TestCase):
    """Test package.xml correctness."""

    def setUp(self):
        pkg_dir = os.path.join(os.path.dirname(__file__), "..")
        self.tree = ET.parse(os.path.join(pkg_dir, "package.xml"))
        self.root = self.tree.getroot()

    def test_package_name(self):
        """Package name is 'rosiwit_app'."""
        name = self.root.find("name")
        self.assertIsNotNone(name)
        self.assertEqual(name.text, "rosiwit_app")

    def test_format_version(self):
        """Package format version is 3."""
        self.assertEqual(self.root.get("format"), "3")

    def test_build_type_is_ament_python(self):
        """Build type is ament_python."""
        export = self.root.find("export")
        self.assertIsNotNone(export)
        build_type = export.find("build_type")
        self.assertIsNotNone(build_type)
        self.assertEqual(build_type.text, "ament_python")

    def test_has_rclpy_dependency(self):
        """Package depends on rclpy."""
        deps = self.root.findall("depend")
        dep_texts = [d.text for d in deps]
        self.assertIn("rclpy", dep_texts)

    def test_has_std_msgs_dependency(self):
        """Package depends on std_msgs."""
        exec_deps = self.root.findall("exec_depend")
        dep_texts = [d.text for d in exec_deps]
        self.assertIn("std_msgs", dep_texts)

    def test_has_nav_msgs_dependency(self):
        """Package depends on nav_msgs."""
        exec_deps = self.root.findall("exec_depend")
        dep_texts = [d.text for d in exec_deps]
        self.assertIn("nav_msgs", dep_texts)

    def test_has_geometry_msgs_dependency(self):
        """Package depends on geometry_msgs."""
        exec_deps = self.root.findall("exec_depend")
        dep_texts = [d.text for d in exec_deps]
        self.assertIn("geometry_msgs", dep_texts)

    def test_has_sensor_msgs_dependency(self):
        """Package depends on sensor_msgs."""
        exec_deps = self.root.findall("exec_depend")
        dep_texts = [d.text for d in exec_deps]
        self.assertIn("sensor_msgs", dep_texts)

    def test_has_std_srvs_dependency(self):
        """Package depends on std_srvs."""
        exec_deps = self.root.findall("exec_depend")
        dep_texts = [d.text for d in exec_deps]
        self.assertIn("std_srvs", dep_texts)


class TestSetupPy(unittest.TestCase):
    """Test setup.py correctness."""

    def setUp(self):
        pkg_dir = os.path.join(os.path.dirname(__file__), "..")
        with open(os.path.join(pkg_dir, "setup.py"), "r") as f:
            self.content = f.read()

    def test_package_name(self):
        """setup.py uses 'rosiwit_app' as package name."""
        self.assertIn("rosiwit_app", self.content)

    def test_entry_points_defined(self):
        """setup.py defines entry_points for console_scripts."""
        self.assertIn("entry_points", self.content)
        self.assertIn("console_scripts", self.content)

    def test_data_files_include_config(self):
        """setup.py includes config files in data_files."""
        self.assertIn("config", self.content)

    def test_data_files_include_launch(self):
        """setup.py includes launch files in data_files."""
        self.assertIn("launch", self.content)


if __name__ == "__main__":
    unittest.main()
