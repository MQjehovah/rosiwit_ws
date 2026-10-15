#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
全覆盖路径规划工具 - 弓字形和回字形算法
基于 ros2_coverage_planner 项目算法逻辑实现
支持 PGM/YAML 地图格式，独立运行无需 ROS2
"""

import numpy as np
import yaml
import cv2
import os
from typing import List, Tuple, Optional
from dataclasses import dataclass
import matplotlib.pyplot as plt


@dataclass
class MapConfig:
    """地图配置"""
    resolution: float
    origin: List[float]
    occupied_thresh: float
    free_thresh: float
    negate: int


@dataclass
class CoverageResult:
    """覆盖规划结果"""
    success: bool
    coverage_rate: float
    path_length: float
    path: List[Tuple[float, float]]
    turn_count: int
    error_message: str = ""


class ZigzagPlanner:
    """弓字形覆盖路径规划器"""
    
    def __init__(self, robot_radius: float = 0.3, coverage_resolution: float = 0.1):
        self.robot_radius = robot_radius
        self.coverage_resolution = coverage_resolution
        
    def plan(self, binary_map: np.ndarray, resolution: float, 
             origin: Tuple[float, float], start_pixel: Tuple[int, int]) -> CoverageResult:
        """
        执行弓字形覆盖规划
        
        参数:
            binary_map: 二值地图 (255=自由, 0=障碍)
            resolution: 地图分辨率 (m/pixel)
            origin: 地图原点 (x, y)
            start_pixel: 起始像素坐标
        """
        height, width = binary_map.shape
        
        # 膨胀地图（考虑机器人半径）
        inflation_radius = int(self.robot_radius / resolution)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, 
                                          (2*inflation_radius+1, 2*inflation_radius+1))
        inflated_map = cv2.erode(binary_map, kernel)
        
        # 选择最优扫描方向
        scan_direction = self._select_optimal_direction(inflated_map)
        
        # 执行扫描线规划
        path_pixels = self._perform_scanline(inflated_map, start_pixel, scan_direction)
        
        if not path_pixels:
            return CoverageResult(False, 0.0, 0.0, [], 0, "规划失败")
        
        # 转换为世界坐标
        path_world = []
        for px, py in path_pixels:
            # PGM坐标系转换：Y轴翻转
            world_x = px * resolution + origin[0]
            world_y = (height - 1 - py) * resolution + origin[1]
            path_world.append((world_x, world_y))
        
        # 计算统计
        coverage_rate, path_length, turn_count = self._calculate_stats(
            path_pixels, inflated_map, resolution)
        
        return CoverageResult(True, coverage_rate, path_length, path_world, turn_count)
    
    def _select_optimal_direction(self, map: np.ndarray) -> int:
        """选择最优扫描方向 (0=水平, 1=垂直)"""
        height, width = map.shape
        
        # 计算每行的自由区域长度变化
        row_lengths = []
        for y in range(height):
            free_pixels = np.sum(map[y, :] == 255)
            row_lengths.append(free_pixels)
        
        row_variance = np.var(row_lengths)
        
        # 计算每列的自由区域长度变化
        col_lengths = []
        for x in range(width):
            free_pixels = np.sum(map[:, x] == 255)
            col_lengths.append(free_pixels)
        
        col_variance = np.var(col_lengths)
        
        # 选择变化较小的方向（减少转弯次数）
        return 0 if row_variance < col_variance else 1
    
    def _perform_scanline(self, map: np.ndarray, start: Tuple[int, int], 
                          direction: int) -> List[Tuple[int, int]]:
        """执行扫描线规划"""
        height, width = map.shape
        path = []
        
        # 步长（像素）
        step = int(self.coverage_resolution / self.robot_radius * 10)  # 简化计算
        
        if direction == 0:  # 水平扫描
            y_coords = self._get_scan_coords(height, start[1], step)
            
            for i, y in enumerate(y_coords):
                if y < 0 or y >= height:
                    continue
                    
                # 找到该行的自由区域段
                row = map[y, :]
                free_segments = self._find_free_segments(row)
                
                if not free_segments:
                    continue
                
                # 选择扫描方向（来回交替）
                if i % 2 == 0:  # 从左到右
                    for seg in free_segments:
                        for x in range(seg[0], seg[1] + 1, 1):
                            if map[y, x] == 255:
                                path.append((x, y))
                else:  # 从右到左
                    for seg in reversed(free_segments):
                        for x in range(seg[1], seg[0] - 1, -1):
                            if map[y, x] == 255:
                                path.append((x, y))
        
        else:  # 垂直扫描
            x_coords = self._get_scan_coords(width, start[0], step)
            
            for i, x in enumerate(x_coords):
                if x < 0 or x >= width:
                    continue
                    
                # 找到该列的自由区域段
                col = map[:, x]
                free_segments = self._find_free_segments(col)
                
                if not free_segments:
                    continue
                
                # 选择扫描方向（来回交替）
                if i % 2 == 0:  # 从上到下
                    for seg in free_segments:
                        for y in range(seg[0], seg[1] + 1, 1):
                            if map[y, x] == 255:
                                path.append((x, y))
                else:  # 从下到上
                    for seg in reversed(free_segments):
                        for y in range(seg[1], seg[0] - 1, -1):
                            if map[y, x] == 255:
                                path.append((x, y))
        
        return path
    
    def _get_scan_coords(self, max_val: int, start: int, step: int) -> List[int]:
        """获取扫描线坐标序列"""
        coords = []
        
        # 从起点向上扫描
        for val in range(start, max_val, step):
            coords.append(val)
        
        # 从起点向下扫描
        for val in range(start - step, 0, -step):
            if val >= 0:
                coords.append(val)
        
        # 合并并排序
        coords.extend([start])
        coords = sorted(set(coords))
        
        return coords
    
    def _find_free_segments(self, line: np.ndarray) -> List[Tuple[int, int]]:
        """找到一行/列中的自由区域段"""
        segments = []
        in_free = False
        start = 0
        
        for i, val in enumerate(line):
            if val == 255 and not in_free:
                in_free = True
                start = i
            elif val != 255 and in_free:
                in_free = False
                segments.append((start, i - 1))
        
        if in_free:
            segments.append((start, len(line) - 1))
        
        return segments
    
    def _calculate_stats(self, path: List[Tuple[int, int]], 
                        map: np.ndarray, resolution: float) -> Tuple[float, float, int]:
        """计算覆盖率、路径长度和转弯次数"""
        # 覆盖率
        visited = set(path)
        total_free = np.sum(map == 255)
        coverage_rate = len(visited) / total_free if total_free > 0 else 0.0
        
        # 路径长度
        path_length = 0.0
        for i in range(len(path) - 1):
            dx = (path[i+1][0] - path[i][0]) * resolution
            dy = (path[i+1][1] - path[i][1]) * resolution
            path_length += np.sqrt(dx*dx + dy*dy)
        
        # 转弯次数
        turn_count = 0
        if len(path) >= 3:
            prev_dx = path[1][0] - path[0][0]
            prev_dy = path[1][1] - path[0][1]
            
            for i in range(2, len(path)):
                dx = path[i][0] - path[i-1][0]
                dy = path[i][1] - path[i-1][1]
                
                # 方向改变即为转弯
                if dx != prev_dx or dy != prev_dy:
                    turn_count += 1
                
                prev_dx, prev_dy = dx, dy
        
        return coverage_rate, path_length, turn_count


class SpiralPlanner:
    """回字形覆盖路径规划器"""
    
    def __init__(self, robot_radius: float = 0.3, coverage_resolution: float = 0.1):
        self.robot_radius = robot_radius
        self.coverage_resolution = coverage_resolution
        
    def plan(self, binary_map: np.ndarray, resolution: float,
             origin: Tuple[float, float], start_pixel: Tuple[int, int]) -> CoverageResult:
        """
        执行回字形覆盖规划
        从外向内螺旋覆盖
        """
        height, width = binary_map.shape
        
        # 膨胀地图
        inflation_radius = int(self.robot_radius / resolution)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                          (2*inflation_radius+1, 2*inflation_radius+1))
        inflated_map = cv2.erode(binary_map, kernel)
        
        # 执行螺旋规划
        path_pixels = self._perform_spiral(inflated_map, start_pixel)
        
        if not path_pixels:
            return CoverageResult(False, 0.0, 0.0, [], 0, "规划失败")
        
        # 转换为世界坐标
        path_world = []
        for px, py in path_pixels:
            world_x = px * resolution + origin[0]
            world_y = (height - 1 - py) * resolution + origin[1]
            path_world.append((world_x, world_y))
        
        # 计算统计
        coverage_rate, path_length, turn_count = self._calculate_stats(
            path_pixels, inflated_map, resolution)
        
        return CoverageResult(True, coverage_rate, path_length, path_world, turn_count)
    
    def _perform_spiral(self, map: np.ndarray, start: Tuple[int, int]) -> List[Tuple[int, int]]:
        """执行螺旋规划"""
        height, width = map.shape
        path = []
        
        # 找到地图边界
        min_x, max_x, min_y, max_y = self._find_bounds(map)
        
        if min_x >= max_x or min_y >= max_y:
            return []
        
        # 从外向内螺旋
        current_x, current_y = min_x, min_y
        
        while min_x <= max_x and min_y <= max_y:
            # 上边（从左到右）
            for x in range(min_x, max_x + 1):
                if map[min_y, x] == 255:
                    path.append((x, min_y))
            
            min_y += 1
            
            # 右边（从上到下）
            for y in range(min_y, max_y + 1):
                if map[y, max_x] == 255:
                    path.append((max_x, y))
            
            max_x -= 1
            
            # 下边（从右到左）
            if min_y <= max_y:
                for x in range(max_x, min_x - 1, -1):
                    if map[max_y, x] == 255:
                        path.append((x, max_y))
                
                max_y -= 1
            
            # 左边（从下到上）
            if min_x <= max_x:
                for y in range(max_y, min_y - 1, -1):
                    if map[y, min_x] == 255:
                        path.append((min_x, y))
                
                min_x += 1
        
        return path
    
    def _find_bounds(self, map: np.ndarray) -> Tuple[int, int, int, int]:
        """找到自由区域的边界"""
        height, width = map.shape
        
        # 查找边界
        min_x, max_x = width, 0
        min_y, max_y = height, 0
        
        for y in range(height):
            for x in range(width):
                if map[y, x] == 255:
                    min_x = min(min_x, x)
                    max_x = max(max_x, x)
                    min_y = min(min_y, y)
                    max_y = max(max_y, y)
        
        return min_x, max_x, min_y, max_y
    
    def _calculate_stats(self, path: List[Tuple[int, int]],
                        map: np.ndarray, resolution: float) -> Tuple[float, float, int]:
        """计算统计信息"""
        # 同 ZigzagPlanner
        visited = set(path)
        total_free = np.sum(map == 255)
        coverage_rate = len(visited) / total_free if total_free > 0 else 0.0
        
        path_length = 0.0
        for i in range(len(path) - 1):
            dx = (path[i+1][0] - path[i][0]) * resolution
            dy = (path[i+1][1] - path[i][1]) * resolution
            path_length += np.sqrt(dx*dx + dy*dy)
        
        turn_count = 0
        if len(path) >= 3:
            prev_dx = path[1][0] - path[0][0]
            prev_dy = path[1][1] - path[0][1]
            
            for i in range(2, len(path)):
                dx = path[i][0] - path[i-1][0]
                dy = path[i][1] - path[i-1][1]
                
                if dx != prev_dx or dy != prev_dy:
                    turn_count += 1
                
                prev_dx, prev_dy = dx, dy
        
        return coverage_rate, path_length, turn_count


def load_map(map_path: str, yaml_path: str) -> Tuple[np.ndarray, MapConfig]:
    """加载PGM地图和YAML配置"""
    # 加载配置
    with open(yaml_path, 'r') as f:
        config = yaml.safe_load(f)
    
    map_config = MapConfig(
        resolution=config.get('resolution', 0.05),
        origin=config.get('origin', [0, 0, 0]),
        occupied_thresh=config.get('occupied_thresh', 0.65),
        free_thresh=config.get('free_thresh', 0.196),
        negate=config.get('negate', 0)
    )
    
    # 加载地图
    grid_map = cv2.imread(map_path, cv2.IMREAD_GRAYSCALE)
    
    if grid_map is None:
        raise ValueError(f"无法加载地图: {map_path}")
    
    # 转换为二值地图
    normalized = grid_map.astype(float) / 255.0
    
    if map_config.negate:
        normalized = 1.0 - normalized
    
    binary_map = np.zeros_like(grid_map)
    binary_map[normalized >= map_config.occupied_thresh] = 255
    
    return binary_map, map_config


def visualize_coverage(map: np.ndarray, path: List[Tuple[int, int]],
                       start: Tuple[int, int], output_path: str,
                       resolution: float, origin: Tuple[float, float],
                       title: str = "Coverage Path"):
    """可视化覆盖路径"""
    height, width = map.shape
    
    # 创建彩色图像
    vis = cv2.cvtColor(map, cv2.COLOR_GRAY2BGR)
    
    # 绘制覆盖路径（绿色）
    for i in range(len(path) - 1):
        x1, y1 = path[i]
        x2, y2 = path[i+1]
        cv2.line(vis, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 1)
    
    # 标记起点（蓝色）
    cv2.circle(vis, (int(start[0]), int(start[1])), 5, (255, 0, 0), -1)
    
    # 标记覆盖区域（淡绿色）
    covered = set(path)
    for px, py in covered:
        vis[int(py), int(px)] = (100, 200, 100)
    
    # 保存图像
    cv2.imwrite(output_path, vis)
    
    # 使用matplotlib创建更详细的可视化
    plt.figure(figsize=(12, 10))
    
    # 转换路径为世界坐标
    path_world_x = [px * resolution + origin[0] for px, py in path]
    path_world_y = [(height - 1 - py) * resolution + origin[1] for px, py in path]
    
    plt.imshow(map, cmap='gray', extent=[
        origin[0], origin[0] + width * resolution,
        origin[1], origin[1] + height * resolution
    ])
    
    plt.plot(path_world_x, path_world_y, 'g-', linewidth=1.5, label='Coverage Path')
    
    start_world_x = start[0] * resolution + origin[0]
    start_world_y = (height - 1 - start[1]) * resolution + origin[1]
    plt.plot(start_world_x, start_world_y, 'bo', markersize=8, label='Start')
    
    plt.title(title)
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    # 保存高清图
    plt.savefig(output_path.replace('.png', '_detailed.png'), dpi=150, bbox_inches='tight')
    plt.close()


def main():
    """主函数"""
    print("="*70)
    print("  全覆盖路径规划工具 - 弓字形和回字形算法")
    print("  基于 ros2_coverage_planner 项目算法")
    print("="*70)
    
    # 地图路径
    map_dir = r"E:\ai\agent\workspace\map"
    pgm_file = os.path.join(map_dir, "map.pgm")
    yaml_file = os.path.join(map_dir, "map.yaml")
    
    # 加载地图
    print("\n[步骤1] 加载地图...")
    binary_map, map_config = load_map(pgm_file, yaml_file)
    
    height, width = binary_map.shape
    print(f"  ✓ 地图尺寸: {width} x {height} 像素")
    print(f"  ✓ 实际尺寸: {width * map_config.resolution:.2f} x {height * map_config.resolution:.2f} 米")
    print(f"  ✓ 分辨率: {map_config.resolution} m/pixel")
    print(f"  ✓ 自由区域: {np.sum(binary_map == 255)} 像素 ({np.sum(binary_map == 255)/(width*height)*100:.1f}%)")
    
    # 找到起始点
    print("\n[步骤2] 选择起始点...")
    free_pixels = np.where(binary_map == 255)
    if len(free_pixels[0]) == 0:
        print("  ✗ 错误: 地图中没有自由区域")
        return
    
    # 选择自由区域的中心作为起点
    start_pixel = (int(np.mean(free_pixels[1])), int(np.mean(free_pixels[0])))
    print(f"  ✓ 起点(像素): ({start_pixel[0]}, {start_pixel[1]})")
    print(f"  ✓ 起点(世界): ({start_pixel[0] * map_config.resolution + map_config.origin[0]:.2f}, "
          f"{(height - 1 - start_pixel[1]) * map_config.resolution + map_config.origin[1]:.2f})")
    
    # 参数配置
    robot_radius = 0.3  # 机器人半径 (米)
    coverage_resolution = 0.1  # 覆盖分辨率 (米)
    
    # 弓字形规划
    print("\n[步骤3] 执行弓字形覆盖规划...")
    zigzag_planner = ZigzagPlanner(robot_radius, coverage_resolution)
    zigzag_result = zigzag_planner.plan(
        binary_map, map_config.resolution,
        (map_config.origin[0], map_config.origin[1]),
        start_pixel
    )
    
    if zigzag_result.success:
        print(f"  ✓ 弓字形规划成功!")
        print(f"  ✓ 覆盖率: {zigzag_result.coverage_rate*100:.2f}%")
        print(f"  ✓ 路径长度: {zigzag_result.path_length:.2f} 米")
        print(f"  ✓ 转弯次数: {zigzag_result.turn_count}")
        print(f"  ✓ 路径点数: {len(zigzag_result.path)}")
        
        # 可视化
        zigzag_img = os.path.join(map_dir, "zigzag_coverage.png")
        zigzag_path_pixels = []
        for wx, wy in zigzag_result.path:
            px = int((wx - map_config.origin[0]) / map_config.resolution)
            py = height - 1 - int((wy - map_config.origin[1]) / map_config.resolution)
            zigzag_path_pixels.append((px, py))
        
        visualize_coverage(binary_map, zigzag_path_pixels, start_pixel,
                          zigzag_img, map_config.resolution,
                          (map_config.origin[0], map_config.origin[1]),
                          "Zigzag Coverage Path")
        print(f"  ✓ 结果保存: {zigzag_img}")
        
        # 保存路径文件
        zigzag_txt = os.path.join(map_dir, "zigzag_path.txt")
        with open(zigzag_txt, 'w') as f:
            f.write(f"# 弓字形覆盖路径\n")
            f.write(f"# 覆盖率: {zigzag_result.coverage_rate*100:.2f}%\n")
            f.write(f"# 路径长度: {zigzag_result.path_length:.2f} 米\n")
            f.write(f"# 转弯次数: {zigzag_result.turn_count}\n")
            f.write(f"# 路径点数: {len(zigzag_result.path)}\n\n")
            
            for i, (x, y) in enumerate(zigzag_result.path):
                f.write(f"{i+1:5d}. ({x:.3f}, {y:.3f})\n")
        
        print(f"  ✓ 路径文件: {zigzag_txt}")
    else:
        print(f"  ✗ 弓字形规划失败: {zigzag_result.error_message}")
    
    # 回字形规划
    print("\n[步骤4] 执行回字形覆盖规划...")
    spiral_planner = SpiralPlanner(robot_radius, coverage_resolution)
    spiral_result = spiral_planner.plan(
        binary_map, map_config.resolution,
        (map_config.origin[0], map_config.origin[1]),
        start_pixel
    )
    
    if spiral_result.success:
        print(f"  ✓ 回字形规划成功!")
        print(f"  ✓ 覆盖率: {spiral_result.coverage_rate*100:.2f}%")
        print(f"  ✓ 路径长度: {spiral_result.path_length:.2f} 米")
        print(f"  ✓ 转弯次数: {spiral_result.turn_count}")
        print(f"  ✓ 路径点数: {len(spiral_result.path)}")
        
        # 可视化
        spiral_img = os.path.join(map_dir, "spiral_coverage.png")
        spiral_path_pixels = []
        for wx, wy in spiral_result.path:
            px = int((wx - map_config.origin[0]) / map_config.resolution)
            py = height - 1 - int((wy - map_config.origin[1]) / map_config.resolution)
            spiral_path_pixels.append((px, py))
        
        visualize_coverage(binary_map, spiral_path_pixels, start_pixel,
                          spiral_img, map_config.resolution,
                          (map_config.origin[0], map_config.origin[1]),
                          "Spiral Coverage Path")
        print(f"  ✓ 结果保存: {spiral_img}")
        
        # 保存路径文件
        spiral_txt = os.path.join(map_dir, "spiral_path.txt")
        with open(spiral_txt, 'w') as f:
            f.write(f"# 回字形覆盖路径\n")
            f.write(f"# 覆盖率: {spiral_result.coverage_rate*100:.2f}%\n")
            f.write(f"# 路径长度: {spiral_result.path_length:.2f} 米\n")
            f.write(f"# 转弯次数: {spiral_result.turn_count}\n")
            f.write(f"# 路径点数: {len(spiral_result.path)}\n\n")
            
            for i, (x, y) in enumerate(spiral_result.path):
                f.write(f"{i+1:5d}. ({x:.3f}, {y:.3f})\n")
        
        print(f"  ✓ 路径文件: {spiral_txt}")
    else:
        print(f"  ✗ 回字形规划失败: {spiral_result.error_message}")
    
    print("\n" + "="*70)
    print("✓ 全覆盖路径规划完成!")
    print("="*70)


if __name__ == "__main__":
    main()