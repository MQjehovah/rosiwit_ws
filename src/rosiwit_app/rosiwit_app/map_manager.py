"""
MapManager - 地图和位置管理器
负责检查/加载/保存地图文件和机器人最后已知位置
"""

import json
import os
import math
from typing import Optional, Dict, Any


class MapManager:
    """管理地图文件的加载、保存和位置持久化"""

    def __init__(self, map_path: str = "/tmp/rosiwit_sim_map", map_file: str = "fast_lio2_map"):
        self.map_path = map_path
        self.map_file = map_file
        self.position_file = os.path.join(map_path, "last_position.json")
        self._ensure_directory()

    def _ensure_directory(self) -> None:
        """确保地图目录存在"""
        os.makedirs(self.map_path, exist_ok=True)

    def has_saved_map(self) -> bool:
        """检查是否存在已保存的栅格地图"""
        yaml_path = self.get_map_yaml_path()
        pgm_path = self.get_map_pgm_path()
        return os.path.isfile(yaml_path) and os.path.isfile(pgm_path)

    def has_saved_pcd(self) -> bool:
        """检查是否存在已保存的点云地图"""
        pcd_path = self.get_map_pcd_path()
        return os.path.isfile(pcd_path)

    def has_saved_position(self) -> bool:
        """检查是否存在已保存的位置"""
        return os.path.isfile(self.position_file)

    def load_position(self) -> Optional[Dict[str, float]]:
        """
        加载上次保存的位置
        Returns:
            dict with {x, y, z, qx, qy, qz, qw} or None
        """
        if not self.has_saved_position():
            return None
        try:
            with open(self.position_file, 'r') as f:
                data = json.load(f)
            return data
        except (json.JSONDecodeError, IOError) as e:
            return None

    def save_position(self, x: float, y: float, z: float,
                      qx: float, qy: float, qz: float, qw: float) -> bool:
        """
        保存当前位置到文件
        Args:
            x, y, z: 位置
            qx, qy, qz, qw: 四元数姿态
        Returns:
            是否保存成功
        """
        data = {
            "x": round(x, 4),
            "y": round(y, 4),
            "z": round(z, 4),
            "qx": round(qx, 6),
            "qy": round(qy, 6),
            "qz": round(qz, 6),
            "qw": round(qw, 6),
            "yaw": round(self._quaternion_to_yaw(qx, qy, qz, qw), 4)
        }
        try:
            self._ensure_directory()
            with open(self.position_file, 'w') as f:
                json.dump(data, f, indent=2)
            return True
        except IOError as e:
            return False

    def get_map_yaml_path(self) -> str:
        """获取地图 YAML 文件完整路径"""
        return os.path.join(self.map_path, f"{self.map_file}.yaml")

    def get_map_pgm_path(self) -> str:
        """获取地图 PGM 文件完整路径"""
        return os.path.join(self.map_path, f"{self.map_file}.pgm")

    def get_map_pcd_path(self) -> str:
        """获取地图 PCD 文件完整路径"""
        return os.path.join(self.map_path, f"{self.map_file}.pcd")

    @staticmethod
    def _quaternion_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
        """四元数转 yaw 角"""
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        return math.atan2(siny_cosp, cosy_cosp)
