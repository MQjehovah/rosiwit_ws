"""
WaypointServer - 航点管理服务
管理命名航点的加载、保存、查询和转换
"""

import os
import yaml
import math
from typing import Optional, Dict, Any, List

from geometry_msgs.msg import PoseStamped


class WaypointServer:
    """管理命名航点，支持从YAML加载和运行时动态管理"""

    def __init__(self, yaml_path: str):
        self.yaml_path = yaml_path
        self._waypoints: Dict[str, Dict[str, Any]] = {}
        self.load_waypoints()

    def load_waypoints(self) -> int:
        """
        从YAML文件加载航点
        Returns:
            加载的航点数量
        """
        if not os.path.isfile(self.yaml_path):
            return 0
        try:
            with open(self.yaml_path, 'r') as f:
                data = yaml.safe_load(f)
            if data and 'waypoints' in data:
                self._waypoints = data['waypoints']
                return len(self._waypoints)
            return 0
        except (yaml.YAMLError, IOError) as e:
            return 0

    def save_waypoints(self) -> bool:
        """保存航点到YAML文件"""
        try:
            os.makedirs(os.path.dirname(self.yaml_path) or '.', exist_ok=True)
            with open(self.yaml_path, 'w') as f:
                yaml.dump({'waypoints': self._waypoints}, f,
                         default_flow_style=False, allow_unicode=True)
            return True
        except IOError as e:
            return False

    def get_waypoint(self, name: str) -> Optional[Dict[str, Any]]:
        """
        获取指定名称的航点
        Args:
            name: 航点名称
        Returns:
            航点字典 {x, y, z, qx, qy, qz, qw, frame_id, description} 或 None
        """
        return self._waypoints.get(name)

    def add_waypoint(self, name: str, x: float, y: float, z: float = 0.0,
                     qx: float = 0.0, qy: float = 0.0, qz: float = 0.0, qw: float = 1.0,
                     frame_id: str = "map", description: str = "") -> bool:
        """添加或更新航点"""
        self._waypoints[name] = {
            "x": float(x), "y": float(y), "z": float(z),
            "qx": float(qx), "qy": float(qy), "qz": float(qz), "qw": float(qw),
            "frame_id": frame_id,
            "description": description
        }
        return self.save_waypoints()

    def remove_waypoint(self, name: str) -> bool:
        """删除航点"""
        if name in self._waypoints:
            del self._waypoints[name]
            return self.save_waypoints()
        return False

    def list_waypoints(self) -> Dict[str, Dict[str, Any]]:
        """返回所有航点"""
        return dict(self._waypoints)

    def list_waypoint_names(self) -> List[str]:
        """返回所有航点名称"""
        return list(self._waypoints.keys())

    def to_pose_stamped(self, name: str) -> Optional[PoseStamped]:
        """
        将航点转换为 ROS2 PoseStamped 消息
        Args:
            name: 航点名称
        Returns:
            PoseStamped 或 None
        """
        wp = self.get_waypoint(name)
        if wp is None:
            return None
        pose = PoseStamped()
        pose.header.frame_id = wp.get("frame_id", "map")
        pose.header.stamp.sec = 0
        pose.header.stamp.nanosec = 0
        pose.pose.position.x = wp["x"]
        pose.pose.position.y = wp["y"]
        pose.pose.position.z = wp.get("z", 0.0)
        pose.pose.orientation.x = wp.get("qx", 0.0)
        pose.pose.orientation.y = wp.get("qy", 0.0)
        pose.pose.orientation.z = wp.get("qz", 0.0)
        pose.pose.orientation.w = wp.get("qw", 1.0)
        return pose

    @staticmethod
    def yaw_to_quaternion(yaw: float) -> tuple:
        """yaw 角转四元数 (qx, qy, qz, qw)"""
        return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))
