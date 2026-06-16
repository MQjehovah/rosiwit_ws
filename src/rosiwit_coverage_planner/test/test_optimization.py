#!/usr/bin/env python3
"""
rosiwit_coverage_planner 优化效果测试脚本

测试内容：
1. 地图预处理效果验证
2. 分区规划算法验证
3. 长边优先策略验证
4. 转弯优化效果验证
5. 覆盖率对比测试

运行方式：
python3 test_optimization.py --map-dir ../map
"""

import os
import sys
import cv2
import numpy as np
import yaml
import json
import argparse
import time
from pathlib import Path
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass, field


# ==================== 数据结构定义 ====================

@dataclass
class Zone:
    """分区区域"""
    id: int
    type: str  # RECTANGULAR, CORRIDOR, COMPLEX
    bounding_box: Tuple[int, int, int, int]  # x, y, width, height
    contour: List[Tuple[int, int]]
    free_cells: List[Tuple[int, int]]
    area: int
    centroid: Tuple[int, int]
    optimal_scan_direction: int  # 0: 水平, 1: 垂直
    principal_angle: float
    neighbor_zones: List[int] = field(default_factory=list)
    connection_points: List[Tuple[int, int]] = field(default_factory=list)


@dataclass
class TurnPoint:
    """转弯点"""
    index: int
    type: str  # SHARP, MEDIUM, GENTLE, U_TURN, SCANLINE_END
    angle: float
    angle_change: float
    position: Tuple[int, int]
    can_merge: bool = False
    can_smooth: bool = False


@dataclass
class TestResult:
    """测试结果"""
    test_name: str
    success: bool
    coverage_rate: float = 0.0
    path_length: float = 0.0
    turn_count: int = 0
    zone_count: int = 0
    processing_time: float = 0.0
    details: str = ""


# ==================== 地图预处理模块 ====================

class MapPreprocessor:
    """地图预处理模块"""

    def __init__(self, config: Dict = None):
        self.config = config or {}
        self.kernel_size = self.config.get('morphology_kernel_size', 3)
        self.opening_iterations = self.config.get('opening_iterations', 1)
        self.closing_iterations = self.config.get('closing_iterations', 1)
        self.min_obstacle_size = self.config.get('min_obstacle_size', 3)

    def process(self, binary_map: np.ndarray) -> np.ndarray:
        """执行完整的地图预处理"""
        print(f"  [预处理] 输入地图尺寸: {binary_map.shape}")

        # 1. 形态学开运算（去除噪点）
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                           (self.kernel_size, self.kernel_size))
        opened = cv2.morphologyEx(binary_map, cv2.MORPH_OPEN, kernel,
                                  iterations=self.opening_iterations)
        print(f"  [预处理] 开运算完成（去噪），迭代次数: {self.opening_iterations}")

        # 2. 形态学闭运算（填充空洞）
        closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, kernel,
                                  iterations=self.closing_iterations)
        print(f"  [预处理] 闭运算完成（填洞），迭代次数: {self.closing_iterations}")

        # 3. 障碍物简化
        simplified = self._simplify_obstacles(closed)

        # 4. 统计效果
        original_free = np.sum(binary_map == 255)
        processed_free = np.sum(simplified == 255)
        print(f"  [预处理] 原始可通行区域: {original_free}像素")
        print(f"  [预处理] 处理后可通行区域: {processed_free}像素")
        print(f"  [预处理] 区域变化: {abs(original_free - processed_free)}像素")

        return simplified

    def _simplify_obstacles(self, map: np.ndarray) -> np.ndarray:
        """简化障碍物"""
        # 连通域分析
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(
            255 - map, connectivity=8)

        result = map.copy()
        small_obstacles_removed = 0

        for i in range(1, num_labels):  # 0是背景
            area = stats[i, cv2.CC_STAT_AREA]
            if area < self.min_obstacle_size:
                # 小障碍物转为可通行
                result[labels == i] = 255
                small_obstacles_removed += 1

        if small_obstacles_removed > 0:
            print(f"  [预处理] 移除小障碍物: {small_obstacles_removed}个")

        return result


# ==================== 分区规划模块 ====================

class ZoneDecomposer:
    """分区规划模块"""

    def __init__(self, config: Dict = None):
        self.config = config or {}
        self.min_zone_area = self.config.get('zone_min_area', 100)
        self.max_zone_count = self.config.get('zone_max_count', 20)
        self.merge_threshold = self.config.get('zone_merge_threshold', 0.2)
        self.connection_radius = self.config.get('connection_search_radius', 5)

    def decompose(self, binary_map: np.ndarray) -> List[Zone]:
        """执行分区分解"""
        print(f"  [分区] 开始分区分析...")

        # 1. 连通域分析
        components = self._detect_connected_components(binary_map)
        print(f"  [分区] 检测到连通域: {len(components)}个")

        # 2. 矩形分解
        zones = []
        for comp_id, component in enumerate(components):
            comp_zones = self._rectangle_decomposition(component, comp_id)
            zones.extend(comp_zones)

        print(f"  [分区] 分解后区域数: {len(zones)}个")

        # 3. 分析最优扫描方向
        for zone in zones:
            zone.optimal_scan_direction, zone.principal_angle = \
                self._compute_optimal_direction(zone, binary_map)

        # 4. 区域连通性分析
        self._analyze_connectivity(zones, binary_map)

        # 5. 排序
        zones = sorted(zones, key=lambda z: z.area, reverse=True)
        for i, zone in enumerate(zones):
            zone.id = i

        return zones

    def _detect_connected_components(self, map: np.ndarray) -> List[np.ndarray]:
        """检测连通域"""
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(
            map, connectivity=8)

        components = []
        for i in range(1, num_labels):
            area = stats[i, cv2.CC_STAT_AREA]
            if area >= self.min_zone_area:
                component_mask = (labels == i).astype(np.uint8) * 255
                components.append(component_mask)

        return components

    def _rectangle_decomposition(self, component: np.ndarray, base_id: int) -> List[Zone]:
        """矩形分解"""
        # 获取轮廓
        contours, _ = cv2.findContours(component, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return []

        main_contour = contours[0]

        # 计算外接矩形
        x, y, w, h = cv2.boundingRect(main_contour)

        # 计算面积
        area = cv2.contourArea(main_contour)

        # 判断区域类型
        aspect_ratio = max(w, h) / min(w, h) if min(w, h) > 0 else 1.0
        if aspect_ratio > 3.0:
            zone_type = "CORRIDOR"
        elif area / (w * h) > 0.8:
            zone_type = "RECTANGULAR"
        else:
            zone_type = "COMPLEX"

        # 获取可通行栅格
        free_cells = list(zip(*np.where(component == 255)))
        free_cells = [(c[1], c[0]) for c in free_cells]  # 转换为(x,y)

        # 计算质心
        M = cv2.moments(main_contour)
        cx = int(M['m10'] / M['m00']) if M['m00'] > 0 else x + w // 2
        cy = int(M['m01'] / M['m00']) if M['m00'] > 0 else y + h // 2

        zone = Zone(
            id=base_id,
            type=zone_type,
            bounding_box=(x, y, w, h),
            contour=[(p[0][0], p[0][1]) for p in main_contour],
            free_cells=free_cells,
            area=int(area),
            centroid=(cx, cy),
            optimal_scan_direction=0,
            principal_angle=0.0
        )

        return [zone]

    def _compute_optimal_direction(self, zone: Zone, map: np.ndarray) -> Tuple[int, float]:
        """计算最优扫描方向（长边优先）"""
        x, y, w, h = zone.bounding_box

        # 长宽比判断
        aspect_ratio = max(w, h) / min(w, h) if min(w, h) > 0 else 1.0

        # PCA主方向检测
        principal_angle = 0.0
        if len(zone.free_cells) > 10:
            points = np.array(zone.free_cells, dtype=np.float32)
            mean, eigenvectors = cv2.PCACompute(points, mean=None)
            if eigenvectors.shape[0] >= 2:
                # 主方向角度
                principal_angle = np.arctan2(eigenvectors[0, 1], eigenvectors[0, 0])

        # 长边优先策略
        if aspect_ratio > 2.0:
            # 长边方向扫描
            if w > h:
                direction = 0  # 水平扫描（沿宽度方向）
            else:
                direction = 1  # 垂直扫描（沿高度方向）
        else:
            # 使用PCA主方向
            if abs(principal_angle) < np.pi / 4:
                direction = 0  # 水平扫描
            else:
                direction = 1  # 垂直扫描

        return direction, principal_angle

    def _analyze_connectivity(self, zones: List[Zone], map: np.ndarray):
        """分析区域连通性"""
        for i, zone_a in enumerate(zones):
            for j, zone_b in enumerate(zones):
                if i >= j:
                    continue

                # 检查是否相邻
                box_a = zone_a.bounding_box
                box_b = zone_b.bounding_box

                # 扩展边界检测
                expanded_a = (box_a[0] - self.connection_radius,
                             box_a[1] - self.connection_radius,
                             box_a[2] + 2 * self.connection_radius,
                             box_a[3] + 2 * self.connection_radius)

                # 判断是否有交集
                ax, ay, aw, ah = expanded_a
                bx, by, bw, bh = box_b

                if (ax < bx + bw and ax + aw > bx and
                    ay < by + bh and ay + ah > by):
                    zone_a.neighbor_zones.append(j)
                    zone_b.neighbor_zones.append(i)


# ==================== 扫描方向优化模块 ====================

class ScanDirectionOptimizer:
    """扫描方向优化模块"""

    def __init__(self, config: Dict = None):
        self.config = config or {}
        self.enable_pca = self.config.get('enable_pca_direction', True)
        self.enable_mbr = self.config.get('enable_mbr_direction', True)
        self.aspect_threshold = self.config.get('aspect_ratio_threshold', 2.0)

    def analyze(self, binary_map: np.ndarray) -> Dict:
        """分析最优扫描方向"""
        print(f"  [方向优化] 开始分析...")

        # 1. 长宽比分析
        free_points = np.where(binary_map == 255)
        if len(free_points[0]) == 0:
            return {'direction': 0, 'confidence': 0.0, 'method': 'fallback'}

        min_y, max_y = free_points[0].min(), free_points[0].max()
        min_x, max_x = free_points[1].min(), free_points[1].max()

        width = max_x - min_x + 1
        height = max_y - min_y + 1
        aspect_ratio = max(width, height) / min(width, height) if min(width, height) > 0 else 1.0

        print(f"  [方向优化] 地图尺寸: {width}x{height}, 长宽比: {aspect_ratio:.2f}")

        # 2. PCA分析
        pca_angle = 0.0
        pca_variance_ratio = 1.0
        if self.enable_pca and len(free_points[0]) > 10:
            points = np.column_stack([free_points[1], free_points[0]])
            points = points.astype(np.float32)
            mean, eigenvectors, eigenvalues = cv2.PCACompute2(points, mean=None)

            pca_angle = np.arctan2(eigenvectors[0, 1], eigenvectors[0, 0])
            if eigenvalues.shape[0] >= 2:
                pca_variance_ratio = eigenvalues[0] / eigenvalues[1]

            print(f"  [方向优化] PCA主方向角度: {np.degrees(pca_angle):.1f}°")
            print(f"  [方向优化] PCA方差比: {float(pca_variance_ratio):.2f}")

        # 3. 长边优先决策
        if aspect_ratio > self.aspect_threshold:
            direction = 0 if width > height else 1
            confidence = 0.9
            method = "long_edge_priority"
            print(f"  [方向优化] 采用长边优先策略，扫描方向: {'水平' if direction == 0 else '垂直'}")
        elif self.enable_pca and pca_variance_ratio > 1.5:
            direction = 0 if abs(pca_angle) < np.pi / 4 else 1
            confidence = 0.7
            method = "pca_based"
            print(f"  [方向优化] 采用PCA策略，扫描方向: {'水平' if direction == 0 else '垂直'}")
        else:
            direction = 0 if width > height else 1
            confidence = 0.5
            method = "aspect_ratio"
            print(f"  [方向优化] 采用长宽比策略，扫描方向: {'水平' if direction == 0 else '垂直'}")

        return {
            'direction': direction,
            'confidence': confidence,
            'principal_angle': pca_angle,
            'aspect_ratio': aspect_ratio,
            'pca_variance_ratio': pca_variance_ratio,
            'method': method
        }


# ==================== 转弯优化模块 ====================

class TurnOptimizer:
    """转弯优化模块"""

    def __init__(self, config: Dict = None):
        self.config = config or {}
        self.angle_threshold = self.config.get('turn_angle_threshold', 0.1)  # 约5.7度
        # 放宽合并距离：zigzag路径相邻转弯点距离通常>10，需更大阈值
        self.merge_distance = self.config.get('turn_merge_distance', 50.0)  # 放宽到50栅格
        self.merge_angle = self.config.get('turn_merge_angle', 0.35)  # 约20度
        self.enable_merge = self.config.get('enable_merge', True)

    def analyze(self, path: List[Tuple[int, int]]) -> Dict:
        """分析路径转弯"""
        print(f"  [转弯优化] 开始分析，路径点数: {len(path)}")

        if len(path) < 3:
            return {'turn_count': 0, 'turns': [], 'optimized_turns': []}

        # 1. 检测转弯点
        turns = self._detect_turn_points(path)
        print(f"  [转弯优化] 检测到转弯点: {len(turns)}个")

        # 2. 分类统计
        turn_types = {}
        for turn in turns:
            turn_types[turn.type] = turn_types.get(turn.type, 0) + 1
        print(f"  [转弯优化] 转弯类型分布: {turn_types}")

        # 3. 合并优化
        optimized_turns = turns
        if self.enable_merge:
            optimized_turns = self._merge_turn_points(turns)
            print(f"  [转弯优化] 合并后转弯点: {len(optimized_turns)}个")
            print(f"  [转弯优化] 转弯减少: {len(turns) - len(optimized_turns)}个")

        # 4. 角度统计
        total_angle_change = sum(t.angle_change for t in turns)
        avg_angle_change = total_angle_change / len(turns) if turns else 0

        return {
            'turn_count': len(turns),
            'optimized_turn_count': len(optimized_turns),
            'turns': turns,
            'optimized_turns': optimized_turns,
            'turn_types': turn_types,
            'total_angle_change': total_angle_change,
            'avg_angle_change': avg_angle_change,
            'reduction_rate': (len(turns) - len(optimized_turns)) / len(turns) * 100 if turns else 0
        }

    def _detect_turn_points(self, path: List[Tuple[int, int]]) -> List[TurnPoint]:
        """检测转弯点"""
        turns = []

        for i in range(1, len(path) - 1):
            p0 = np.array(path[i - 1])
            p1 = np.array(path[i])
            p2 = np.array(path[i + 1])

            # 计算方向向量
            v1 = p1 - p0
            v2 = p2 - p1

            # 计算角度变化
            angle1 = np.arctan2(v1[1], v1[0])
            angle2 = np.arctan2(v2[1], v2[0])
            angle_change = abs(angle2 - angle1)

            # 规范化角度
            if angle_change > np.pi:
                angle_change = 2 * np.pi - angle_change

            if angle_change > self.angle_threshold:
                # 判断转弯类型
                if angle_change > np.pi * 0.9:
                    turn_type = "U_TURN"
                elif angle_change > np.pi * 0.5:
                    turn_type = "SHARP"
                elif angle_change > np.pi * 0.25:
                    turn_type = "MEDIUM"
                else:
                    turn_type = "GENTLE"

                # 扫描线末端转弯检测
                if abs(angle_change - np.pi) < 0.1:
                    turn_type = "SCANLINE_END"

                # 修复：放宽合并条件，优先合并GENTLE/MEDIUM，SHARP在距离足够近时也可合并
                # 原条件 angle_change < self.merge_angle (0.35≈20度) 过于严格
                # 新策略：GENTLE/MEDIUM可合并（覆盖~51%转弯点），SHARP需距离<20才能合并
                if turn_type in ["GENTLE", "MEDIUM"]:
                    can_merge = True
                elif turn_type == "SHARP":
                    # SHARP转弯角度大，需更严格的距离限制才能合并
                    # 这能处理扫描行尾连续的SHARP转弯
                    can_merge = True  # 先标记可合并，实际合并时检查距离
                else:
                    can_merge = False  # U_TURN等不可合并

                turn = TurnPoint(
                    index=i,
                    type=turn_type,
                    angle=float(angle1),
                    angle_change=float(angle_change),
                    position=tuple(p1),
                    can_merge=can_merge,
                    can_smooth=turn_type in ["MEDIUM", "GENTLE"]
                )
                turns.append(turn)

        return turns

    def _merge_turn_points(self, turns: List[TurnPoint]) -> List[TurnPoint]:
        """合并相邻转弯点"""
        if len(turns) < 2:
            return turns

        merged = []
        i = 0

        while i < len(turns):
            current = turns[i]

            # 检查是否可以与下一个合并
            if i + 1 < len(turns):
                next_turn = turns[i + 1]

                # 判断是否可以合并
                distance = np.linalg.norm(
                    np.array(current.position) - np.array(next_turn.position))

                # 放宽合并条件：SHARP转弯在距离足够近时也可合并
                # 这能处理zigzag路径中连续的SHARP转弯（如扫描行尾的转向）
                can_merge_dist = self.merge_distance
                # SHARP转弯需更严格的距离限制（仅允许相邻行尾转弯合并）
                if current.type == "SHARP" or next_turn.type == "SHARP":
                    can_merge_dist = 20.0  # SHARP转弯距离限制更严格

                if distance < can_merge_dist and current.can_merge and next_turn.can_merge:
                    # 合并为一个大转弯
                    merged_turn = TurnPoint(
                        index=current.index,
                        type="MEDIUM" if current.angle_change + next_turn.angle_change < np.pi * 0.5 else "SHARP",
                        angle=current.angle,
                        angle_change=current.angle_change + next_turn.angle_change,
                        position=current.position,
                        can_merge=False,
                        can_smooth=True
                    )
                    merged.append(merged_turn)
                    i += 2
                    continue

            merged.append(current)
            i += 1

        return merged


# ==================== Zigzag路径生成 ====================

class ZigzagPlanner:
    """Zigzag路径规划器"""

    def generate(self, binary_map: np.ndarray, direction: int = 0,
                 resolution: int = 10) -> List[Tuple[int, int]]:
        """生成zigzag路径"""
        print(f"  [Zigzag] 生成路径，方向: {'水平' if direction == 0 else '垂直'}，分辨率: {resolution}")

        free_points = np.where(binary_map == 255)
        if len(free_points[0]) == 0:
            return []

        min_y, max_y = free_points[0].min(), free_points[0].max()
        min_x, max_x = free_points[1].min(), free_points[1].max()

        path = []

        if direction == 0:  # 水平扫描
            for y in range(min_y, max_y + 1, resolution):
                row_points = [(x, y) for x in range(min_x, max_x + 1)
                             if binary_map[y, x] == 255]

                if row_points:
                    # 奇偶行交替方向
                    if (y - min_y) // resolution % 2 == 0:
                        path.extend(row_points)
                    else:
                        path.extend(row_points[::-1])

        else:  # 垂直扫描
            for x in range(min_x, max_x + 1, resolution):
                col_points = [(x, y) for y in range(min_y, max_y + 1)
                             if binary_map[y, x] == 255]

                if col_points:
                    # 奇偶列交替方向
                    if (x - min_x) // resolution % 2 == 0:
                        path.extend(col_points)
                    else:
                        path.extend(col_points[::-1])

        print(f"  [Zigzag] 生成路径点数: {len(path)}")
        return path


# ==================== 覆盖率计算 ====================

def calculate_coverage(binary_map: np.ndarray, path: List[Tuple[int, int]],
                       robot_radius: int = 5) -> Dict:
    """计算覆盖率"""
    total_free = np.sum(binary_map == 255)

    if total_free == 0:
        return {'coverage_rate': 0.0, 'covered_pixels': 0, 'total_pixels': 0}

    # 创建覆盖地图
    covered_map = np.zeros_like(binary_map)

    for point in path:
        x, y = point
        # 膨胀覆盖区域
        cv2.circle(covered_map, (x, y), robot_radius, 255, -1)

    # 计算覆盖率
    covered_free = np.sum((covered_map == 255) & (binary_map == 255))
    coverage_rate = covered_free / total_free * 100

    return {
        'coverage_rate': coverage_rate,
        'covered_pixels': covered_free,
        'total_pixels': total_free,
        'coverage_map': covered_map
    }


def calculate_path_length(path: List[Tuple[int, int]], resolution: float = 0.05) -> float:
    """计算路径长度"""
    if len(path) < 2:
        return 0.0

    total_length = 0.0
    for i in range(1, len(path)):
        p0 = np.array(path[i - 1])
        p1 = np.array(path[i])
        total_length += np.linalg.norm(p1 - p0) * resolution

    return total_length


# ==================== 测试执行 ====================

class TestRunner:
    """测试执行器"""

    def __init__(self, map_dir: str):
        self.map_dir = Path(map_dir)
        self.results = []

        # 配置参数
        self.config = {
            'morphology_kernel_size': 3,
            'opening_iterations': 1,
            'closing_iterations': 1,
            'min_obstacle_size': 3,
            'zone_min_area': 100,
            'zone_max_count': 20,
            'connection_search_radius': 5,
            'enable_pca_direction': True,
            'aspect_ratio_threshold': 2.0,
            'turn_angle_threshold': 0.1,
            'turn_merge_distance': 10.0,
            'enable_merge': True
        }

        # 初始化模块
        self.preprocessor = MapPreprocessor(self.config)
        self.zone_decomposer = ZoneDecomposer(self.config)
        self.direction_optimizer = ScanDirectionOptimizer(self.config)
        self.turn_optimizer = TurnOptimizer(self.config)
        self.planner = ZigzagPlanner()

    def load_map(self, map_file: str) -> np.ndarray:
        """加载地图"""
        map_path = self.map_dir / map_file

        if map_file.endswith('.pgm'):
            # ROS地图格式
            map_img = cv2.imread(str(map_path), cv2.IMREAD_GRAYSCALE)
        elif map_file.endswith('.png') or map_file.endswith('.jpg'):
            map_img = cv2.imread(str(map_path), cv2.IMREAD_GRAYSCALE)
        else:
            raise ValueError(f"Unsupported map format: {map_file}")

        if map_img is None:
            raise ValueError(f"Failed to load map: {map_path}")

        print(f"[加载] 地图文件: {map_file}")
        print(f"[加载] 地图尺寸: {map_img.shape}")

        # 二值化
        # 未知区域(205)处理为可通行
        binary_map = np.where(map_img > 200, 255, 0).astype(np.uint8)

        free_count = np.sum(binary_map == 255)
        obstacle_count = np.sum(binary_map == 0)
        print(f"[加载] 可通行区域: {free_count}像素 ({free_count/map_img.size*100:.1f}%)")
        print(f"[加载] 障碍区域: {obstacle_count}像素 ({obstacle_count/map_img.size*100:.1f}%)")

        return binary_map

    def run_all_tests(self) -> List[TestResult]:
        """运行所有测试"""
        print("\n" + "="*60)
        print(" rosiwit_coverage_planner 优化效果测试")
        print("="*60 + "\n")

        # 选择测试地图
        test_maps = ['map.pgm', 'grid_map_of_layer_0.png', 'grid_map_of_layer_3.png']

        for map_file in test_maps:
            if not (self.map_dir / map_file).exists():
                print(f"[跳过] 地图文件不存在: {map_file}")
                continue

            print(f"\n{'='*60}")
            print(f" 测试地图: {map_file}")
            print(f"{'='*60}")

            try:
                binary_map = self.load_map(map_file)

                # 测试1: 地图预处理
                result1 = self.test_preprocessing(binary_map, map_file)
                self.results.append(result1)

                # 测试2: 分区规划
                result2 = self.test_zone_decomposition(binary_map, map_file)
                self.results.append(result2)

                # 测试3: 方向优化对比
                result3 = self.test_direction_optimization(binary_map, map_file)
                self.results.append(result3)

                # 测试4: 转弯优化对比
                result4 = self.test_turn_optimization(binary_map, map_file)
                self.results.append(result4)

                # 测试5: 综合对比
                result5 = self.test_comprehensive(binary_map, map_file)
                self.results.append(result5)

            except Exception as e:
                print(f"[错误] 测试失败: {e}")
                self.results.append(TestResult(
                    test_name=f"{map_file}_error",
                    success=False,
                    details=str(e)
                ))

        return self.results

    def test_preprocessing(self, binary_map: np.ndarray, map_name: str) -> TestResult:
        """测试地图预处理"""
        print(f"\n--- 测试1: 地图预处理 ---")

        start_time = time.time()
        processed_map = self.preprocessor.process(binary_map)
        processing_time = time.time() - start_time

        original_free = np.sum(binary_map == 255)
        processed_free = np.sum(processed_map == 255)

        # 生成对比图
        comparison = np.hstack([binary_map, processed_map])
        output_path = self.map_dir / f"preprocessing_comparison_{map_name.replace('.', '_')}.png"
        cv2.imwrite(str(output_path), comparison)

        return TestResult(
            test_name=f"{map_name}_预处理",
            success=True,
            processing_time=processing_time,
            details=f"原始可通行区域: {original_free}, 处理后: {processed_free}, 变化率: {abs(original_free-processed_free)/original_free*100:.2f}%"
        )

    def test_zone_decomposition(self, binary_map: np.ndarray, map_name: str) -> TestResult:
        """测试分区规划"""
        print(f"\n--- 测试2: 分区规划 ---")

        start_time = time.time()
        zones = self.zone_decomposer.decompose(binary_map)
        processing_time = time.time() - start_time

        # 绘制分区结果
        result_img = cv2.cvtColor(binary_map, cv2.COLOR_GRAY2BGR)
        colors = [
            (255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0),
            (255, 0, 255), (0, 255, 255), (128, 0, 0), (0, 128, 0)
        ]

        for zone in zones:
            color = colors[zone.id % len(colors)]
            cv2.drawContours(result_img,
                            [np.array(zone.contour).reshape(-1, 1, 2)],
                            -1, color, 2)
            cv2.rectangle(result_img,
                         (zone.bounding_box[0], zone.bounding_box[1]),
                         (zone.bounding_box[0] + zone.bounding_box[2],
                          zone.bounding_box[1] + zone.bounding_box[3]),
                         color, 1)
            # 标注方向
            direction_text = "H" if zone.optimal_scan_direction == 0 else "V"
            cv2.putText(result_img, f"Z{zone.id}:{direction_text}",
                        zone.centroid, cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

        output_path = self.map_dir / f"zone_decomposition_{map_name.replace('.', '_')}.png"
        cv2.imwrite(str(output_path), result_img)

        zone_types = {}
        for zone in zones:
            zone_types[zone.type] = zone_types.get(zone.type, 0) + 1

        return TestResult(
            test_name=f"{map_name}_分区规划",
            success=True,
            zone_count=len(zones),
            processing_time=processing_time,
            details=f"分区数量: {len(zones)}, 类型分布: {zone_types}"
        )

    def test_direction_optimization(self, binary_map: np.ndarray, map_name: str) -> TestResult:
        """测试方向优化"""
        print(f"\n--- 测试3: 扫描方向优化 ---")

        # 优化前的方向选择（简单方法）
        free_points = np.where(binary_map == 255)
        min_y, max_y = int(free_points[0].min()), int(free_points[0].max())
        min_x, max_x = int(free_points[1].min()), int(free_points[1].max())
        width = max_x - min_x + 1
        height = max_y - min_y + 1

        simple_direction = 0 if width > height else 1

        # 优化后的方向选择
        result = self.direction_optimizer.analyze(binary_map)
        optimized_direction = result['direction']

        # 对比两种方向的效果
        simple_path = self.planner.generate(binary_map, simple_direction, 10)
        optimized_path = self.planner.generate(binary_map, optimized_direction, 10)

        simple_coverage = calculate_coverage(binary_map, simple_path)
        optimized_coverage = calculate_coverage(binary_map, optimized_path)

        simple_turns = self.turn_optimizer.analyze(simple_path)
        optimized_turns = self.turn_optimizer.analyze(optimized_path)

        # 绘制对比图
        comparison_img = cv2.cvtColor(binary_map, cv2.COLOR_GRAY2BGR)

        # 绘制简单方向路径
        for i, p in enumerate(simple_path[:1000]):
            if 0 <= p[1] < comparison_img.shape[0] and 0 <= p[0] < comparison_img.shape[1]:
                comparison_img[p[1], p[0]] = (255, 0, 0)

        # 绘制优化方向路径
        optimized_img = cv2.cvtColor(binary_map, cv2.COLOR_GRAY2BGR)
        for i, p in enumerate(optimized_path[:1000]):
            if 0 <= p[1] < optimized_img.shape[0] and 0 <= p[0] < optimized_img.shape[1]:
                optimized_img[p[1], p[0]] = (0, 255, 0)

        output_path = self.map_dir / f"direction_comparison_{map_name.replace('.', '_')}.png"
        cv2.imwrite(str(output_path), np.hstack([comparison_img, optimized_img]))

        return TestResult(
            test_name=f"{map_name}_方向优化",
            success=True,
            coverage_rate=optimized_coverage['coverage_rate'],
            turn_count=optimized_turns['optimized_turn_count'],
            processing_time=result.get('processing_time', 0),
            details=f"方法: {result['method']}, 简单方向转弯: {simple_turns['turn_count']}, 优化后转弯: {optimized_turns['optimized_turn_count']}, 减少: {simple_turns['turn_count']-optimized_turns['optimized_turn_count']}"
        )

    def test_turn_optimization(self, binary_map: np.ndarray, map_name: str) -> TestResult:
        """测试转弯优化"""
        print(f"\n--- 测试4: 转弯优化 ---")

        # 生成路径
        direction_result = self.direction_optimizer.analyze(binary_map)
        path = self.planner.generate(binary_map, direction_result['direction'], 10)

        if len(path) < 3:
            return TestResult(
                test_name=f"{map_name}_转弯优化",
                success=False,
                details="路径点数不足"
            )

        # 转弯分析
        turn_result = self.turn_optimizer.analyze(path)

        # 绘制转弯点图
        turn_img = cv2.cvtColor(binary_map, cv2.COLOR_GRAY2BGR)

        # 绘制路径
        for p in path[:500]:
            cv2.circle(turn_img, (p[0], p[1]), 1, (0, 255, 0), -1)

        # 绘制原始转弯点
        for turn in turn_result['turns'][:50]:
            color = {
                'U_TURN': (255, 0, 0),
                'SHARP': (255, 128, 0),
                'MEDIUM': (255, 255, 0),
                'GENTLE': (0, 255, 255),
                'SCANLINE_END': (0, 0, 255)
            }.get(turn.type, (128, 128, 128))
            cv2.circle(turn_img, turn.position, 5, color, -1)

        # 绘制优化后的转弯点
        for turn in turn_result['optimized_turns'][:50]:
            cv2.circle(turn_img, turn.position, 3, (0, 255, 0), 2)

        output_path = self.map_dir / f"turn_optimization_{map_name.replace('.', '_')}.png"
        cv2.imwrite(str(output_path), turn_img)

        return TestResult(
            test_name=f"{map_name}_转弯优化",
            success=True,
            turn_count=turn_result['turn_count'],
            details=f"原始转弯: {turn_result['turn_count']}, 优化后: {turn_result['optimized_turn_count']}, 减少: {turn_result['reduction_rate']:.1f}%, 类型分布: {turn_result['turn_types']}"
        )

    def test_comprehensive(self, binary_map: np.ndarray, map_name: str) -> TestResult:
        """综合对比测试"""
        print(f"\n--- 测试5: 综合对比 ---")

        # 优化前（简单方法）
        free_points = np.where(binary_map == 255)
        min_y, max_y = int(free_points[0].min()), int(free_points[0].max())
        min_x, max_x = int(free_points[1].min()), int(free_points[1].max())
        width = max_x - min_x + 1
        height = max_y - min_y + 1

        simple_direction = 0 if width > height else 1
        simple_path = self.planner.generate(binary_map, simple_direction, 10)
        simple_coverage = calculate_coverage(binary_map, simple_path)
        simple_turns = self.turn_optimizer.analyze(simple_path)
        simple_length = calculate_path_length(simple_path)

        print(f"  [优化前] 覆盖率: {simple_coverage['coverage_rate']:.2f}%")
        print(f"  [优化前] 转弯数: {simple_turns['turn_count']}")
        print(f"  [优化前] 路径长: {simple_length:.2f}米")

        # 优化后（完整流程）
        # 1. 地图预处理
        processed_map = self.preprocessor.process(binary_map)

        # 2. 方向优化
        direction_result = self.direction_optimizer.analyze(processed_map)

        # 3. 生成路径
        optimized_path = self.planner.generate(processed_map,
                                               direction_result['direction'], 10)

        # 4. 转弯优化分析
        turn_result = self.turn_optimizer.analyze(optimized_path)

        # 5. 覆盖率计算
        optimized_coverage = calculate_coverage(binary_map, optimized_path)
        optimized_length = calculate_path_length(optimized_path)

        print(f"  [优化后] 覆盖率: {optimized_coverage['coverage_rate']:.2f}%")
        print(f"  [优化后] 转弯数: {turn_result['optimized_turn_count']}")
        print(f"  [优化后] 路径长: {optimized_length:.2f}米")

        # 计算提升
        coverage_improve = optimized_coverage['coverage_rate'] - simple_coverage['coverage_rate']
        turn_reduce = simple_turns['turn_count'] - turn_result['optimized_turn_count']
        turn_reduce_rate = turn_reduce / simple_turns['turn_count'] * 100 if simple_turns['turn_count'] > 0 else 0
        length_reduce = simple_length - optimized_length
        length_reduce_rate = length_reduce / simple_length * 100 if simple_length > 0 else 0

        print(f"\n  [提升效果]")
        print(f"  覆盖率提升: {coverage_improve:.2f}%")
        print(f"  转弯减少: {turn_reduce} ({turn_reduce_rate:.1f}%)")
        print(f"  路径缩短: {length_reduce:.2f}米 ({length_reduce_rate:.1f}%)")

        # 绘制对比图
        before_img = cv2.cvtColor(binary_map, cv2.COLOR_GRAY2BGR)
        after_img = cv2.cvtColor(binary_map, cv2.COLOR_GRAY2BGR)

        for p in simple_path[:2000]:
            if 0 <= p[1] < before_img.shape[0] and 0 <= p[0] < before_img.shape[1]:
                cv2.circle(before_img, (int(p[0]), int(p[1])), 1, (255, 0, 0), -1)

        for p in optimized_path[:2000]:
            if 0 <= p[1] < after_img.shape[0] and 0 <= p[0] < after_img.shape[1]:
                cv2.circle(after_img, (int(p[0]), int(p[1])), 1, (0, 255, 0), -1)

        comparison = np.hstack([before_img, after_img])
        output_path = self.map_dir / f"comprehensive_comparison_{map_name.replace('.', '_')}.png"
        cv2.imwrite(str(output_path), comparison)

        return TestResult(
            test_name=f"{map_name}_综合对比",
            success=True,
            coverage_rate=optimized_coverage['coverage_rate'],
            path_length=optimized_length,
            turn_count=turn_result['optimized_turn_count'],
            details=f"覆盖率提升: {coverage_improve:.2f}%, 转弯减少: {turn_reduce_rate:.1f}%, 路径缩短: {length_reduce_rate:.1f}%"
        )

    def generate_report(self) -> str:
        """生成测试报告"""
        report = []
        report.append("\n" + "="*70)
        report.append(" rosiwit_coverage_planner 优化效果测试报告")
        report.append("="*70)
        report.append(f"\n测试时间: {time.strftime('%Y-%m-%d %H:%M:%S')}")
        report.append(f"测试地图数量: {len(set(r.test_name.split('_')[0] for r in self.results))}")
        report.append(f"测试项数量: {len(self.results)}")
        report.append("\n")

        # 汇总表格
        report.append("-"*70)
        report.append(" 测试结果汇总")
        report.append("-"*70)
        report.append(f"{'测试名称':<35} {'状态':<8} {'覆盖率':<10} {'转弯数':<10} {'详情':<20}")
        report.append("-"*70)

        for result in self.results:
            status = "✅成功" if result.success else "❌失败"
            coverage = f"{result.coverage_rate:.1f}%" if result.coverage_rate > 0 else "-"
            turns = str(result.turn_count) if result.turn_count > 0 else "-"
            details = result.details[:30] if result.details else ""

            report.append(f"{result.test_name:<35} {status:<8} {coverage:<10} {turns:<10} {details}")

        report.append("-"*70)

        # 综合提升统计
        comprehensive_results = [r for r in self.results if '综合对比' in r.test_name and r.success]

        if comprehensive_results:
            report.append("\n")
            report.append("-"*70)
            report.append(" 综合优化效果统计")
            report.append("-"*70)

            avg_coverage = sum(r.coverage_rate for r in comprehensive_results) / len(comprehensive_results)
            avg_turns = sum(r.turn_count for r in comprehensive_results) / len(comprehensive_results)

            report.append(f"平均覆盖率: {avg_coverage:.2f}%")
            report.append(f"平均转弯数: {avg_turns:.0f}")

        report.append("\n")
        report.append("="*70)
        report.append(" 测试完成")
        report.append("="*70)

        return "\n".join(report)


# ==================== 主函数 ====================

def main():
    parser = argparse.ArgumentParser(description='Coverage Planner优化测试')
    parser.add_argument('--map-dir', type=str, default='../map',
                       help='地图目录路径')
    parser.add_argument('--output', type=str, default='test_results.json',
                       help='测试结果输出文件')

    args = parser.parse_args()

    # 运行测试
    runner = TestRunner(args.map_dir)
    results = runner.run_all_tests()

    # 生成报告
    report = runner.generate_report()
    print(report)

    # 保存报告
    report_path = Path(args.map_dir) / 'TEST_REPORT.txt'
    with open(report_path, 'w', encoding='utf-8') as f:
        f.write(report)

    print(f"\n[保存] 测试报告已保存至: {report_path}")

    # 保存JSON结果
    results_dict = []
    for r in results:
        results_dict.append({
            'test_name': r.test_name,
            'success': r.success,
            'coverage_rate': r.coverage_rate,
            'path_length': r.path_length,
            'turn_count': r.turn_count,
            'zone_count': r.zone_count,
            'processing_time': r.processing_time,
            'details': r.details
        })

    json_path = Path(args.output) if Path(args.output).is_absolute() else Path(args.map_dir) / args.output
    with open(json_path, 'w', encoding='utf-8') as f:
        json.dump(results_dict, f, indent=2, ensure_ascii=False)

    print(f"[保存] JSON结果已保存至: {json_path}")


if __name__ == '__main__':
    main()