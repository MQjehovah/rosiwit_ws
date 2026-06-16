#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
路径规划脚本 - 使用A*算法在栅格地图上进行路径规划
读取ROS标准的PGM地图文件，实现从起点到终点的路径规划
"""

import numpy as np
import heapq
import yaml
import cv2
import os
from typing import List, Tuple, Optional
from dataclasses import dataclass


@dataclass
class MapConfig:
    """地图配置信息"""
    resolution: float  # 分辨率 (m/pixel)
    origin: List[float]  # 地图原点 [x, y, theta]
    occupied_thresh: float  # 占用阈值
    free_thresh: float  # 自由阈值
    negate: int  # 是否反转


class PathPlanner:
    """路径规划器"""

    def __init__(self, map_path: str, yaml_path: str):
        self.map_path = map_path
        self.yaml_path = yaml_path
        self.grid_map = None
        self.map_config = None
        self.height = 0
        self.width = 0

    def load_map(self) -> bool:
        """加载地图文件"""
        print("="*60)
        print("步骤1: 加载地图文件")
        print("="*60)

        # 加载YAML配置
        with open(self.yaml_path, 'r') as f:
            config = yaml.safe_load(f)

        self.map_config = MapConfig(
            resolution=config.get('resolution', 0.05),
            origin=config.get('origin', [0, 0, 0]),
            occupied_thresh=config.get('occupied_thresh', 0.65),
            free_thresh=config.get('free_thresh', 0.196),
            negate=config.get('negate', 0)
        )

        print(f"✓ 地图配置已加载:")
        print(f"  - 分辨率: {self.map_config.resolution} m/pixel")
        print(f"  - 原点: {self.map_config.origin}")
        print(f"  - 占用阈值: {self.map_config.occupied_thresh}")
        print(f"  - 自由阈值: {self.map_config.free_thresh}")

        # 加载PGM图像
        if not os.path.exists(self.map_path):
            print(f"✗ 错误: 地图文件不存在: {self.map_path}")
            return False

        # 使用OpenCV读取PGM文件
        self.grid_map = cv2.imread(self.map_path, cv2.IMREAD_GRAYSCALE)
        if self.grid_map is None:
            print(f"✗ 错误: 无法读取地图文件: {self.map_path}")
            return False

        self.height, self.width = self.grid_map.shape
        print(f"✓ 地图已加载:")
        print(f"  - 尺寸: {self.width} x {self.height} 像素")
        print(f"  - 实际尺寸: {self.width * self.map_config.resolution:.2f} x {self.height * self.map_config.resolution:.2f} 米")
        print(f"  - 像素值范围: {self.grid_map.min()} - {self.grid_map.max()}")

        return True

    def preprocess_map(self) -> np.ndarray:
        """预处理地图，转换为二值栅格"""
        print("\n" + "="*60)
        print("步骤2: 预处理地图")
        print("="*60)

        # 归一化到 [0, 1]
        normalized_map = self.grid_map.astype(float) / 255.0

        # 根据阈值分类
        # 0 = 障碍物 (黑色), 1 = 自由 (白色)
        if self.map_config.negate:
            normalized_map = 1.0 - normalized_map

        # 创建二值地图: 0=障碍物, 255=自由区域
        binary_map = np.zeros_like(self.grid_map)

        # 自由区域
        free_mask = normalized_map >= self.map_config.occupied_thresh
        binary_map[free_mask] = 255

        # 统计
        obstacle_count = np.sum(binary_map == 0)
        free_count = np.sum(binary_map == 255)
        total = self.width * self.height

        print(f"✓ 地图预处理完成:")
        print(f"  - 自由区域: {free_count} 像素 ({free_count/total*100:.1f}%)")
        print(f"  - 障碍物区域: {obstacle_count} 像素 ({obstacle_count/total*100:.1f}%)")

        return binary_map

    def world_to_map(self, world_x: float, world_y: float) -> Tuple[int, int]:
        """将世界坐标转换为地图坐标"""
        map_x = int((world_x - self.map_config.origin[0]) / self.map_config.resolution)
        map_y = int((world_y - self.map_config.origin[1]) / self.map_config.resolution)
        # PGM图像坐标系: 原点在左下角，需要翻转Y轴
        map_y = self.height - 1 - map_y
        return (map_x, map_y)

    def map_to_world(self, map_x: int, map_y: int) -> Tuple[float, float]:
        """将地图坐标转换为世界坐标"""
        # 翻转Y轴
        map_y_flipped = self.height - 1 - map_y
        world_x = map_x * self.map_config.resolution + self.map_config.origin[0]
        world_y = map_y_flipped * self.map_config.resolution + self.map_config.origin[1]
        return (world_x, world_y)

    def is_valid(self, x: int, y: int, binary_map: np.ndarray) -> bool:
        """检查坐标是否有效且可通行"""
        if x < 0 or x >= self.width or y < 0 or y >= self.height:
            return False
        return binary_map[y, x] == 255

    def find_nearest_free(self, x: int, y: int, binary_map: np.ndarray,
                         max_radius: int = 50) -> Optional[Tuple[int, int]]:
        """找到最近的自由区域点"""
        if self.is_valid(x, y, binary_map):
            return (x, y)

        # 在圆形区域内搜索最近的有效点
        for radius in range(1, max_radius):
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    if abs(dx) == radius or abs(dy) == radius:
                        new_x, new_y = x + dx, y + dy
                        if self.is_valid(new_x, new_y, binary_map):
                            return (new_x, new_y)

        return None

    def find_free_region_centers(self, binary_map: np.ndarray, num_regions: int = 2) -> List[Tuple[int, int]]:
        """找到地图中主要自由区域的中心点"""
        try:
            from scipy import ndimage
        except ImportError:
            print("⚠ 未安装scipy，使用简化方法查找自由区域...")
            return self._find_free_regions_simple(binary_map, num_regions)

        # 标记连通区域
        labeled, num_features = ndimage.label(binary_map == 255)

        if num_features == 0:
            return []

        # 找到每个区域的质心
        centers = []
        for i in range(1, num_features + 1):
            region_mask = (labeled == i)
            if np.sum(region_mask) < 100:  # 忽略太小的区域
                continue

            # 计算质心
            y_coords, x_coords = np.where(region_mask)
            center_x = int(np.mean(x_coords))
            center_y = int(np.mean(y_coords))

            # 确保中心点在自由区域内
            if self.is_valid(center_x, center_y, binary_map):
                centers.append((center_x, center_y))

        # 按区域大小排序，返回最大的几个
        if len(centers) > num_regions:
            # 计算每个中心点所属区域的大小
            region_sizes = []
            for cx, cy in centers:
                region_id = labeled[cy, cx]
                size = np.sum(labeled == region_id)
                region_sizes.append((size, (cx, cy)))

            region_sizes.sort(reverse=True)
            centers = [pos for size, pos in region_sizes[:num_regions]]

        return centers

    def _find_free_regions_simple(self, binary_map: np.ndarray, num_regions: int) -> List[Tuple[int, int]]:
        """简化版自由区域查找（不依赖scipy）"""
        # 网格化搜索
        grid_size = 50
        centers = []

        for y in range(0, self.height, grid_size):
            for x in range(0, self.width, grid_size):
                # 检查网格中心是否为自由区域
                cx, cy = x + grid_size // 2, y + grid_size // 2
                if self.is_valid(cx, cy, binary_map):
                    centers.append((cx, cy))
                    if len(centers) >= num_regions:
                        return centers

        return centers

    def heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        """启发式函数 - 欧几里得距离"""
        return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

    def a_star(self, start: Tuple[int, int], goal: Tuple[int, int],
               binary_map: np.ndarray) -> Optional[List[Tuple[int, int]]]:
        """A*路径规划算法"""
        print("\n" + "="*60)
        print("步骤3: 执行A*路径规划")
        print("="*60)
        print(f"原始起点(像素): {start}")
        print(f"原始终点(像素): {goal}")

        # 检查并调整起点
        if not self.is_valid(start[0], start[1], binary_map):
            print(f"⚠ 起点 {start} 位于障碍物上，正在寻找最近的自由区域...")
            new_start = self.find_nearest_free(start[0], start[1], binary_map)
            if new_start is None:
                print(f"✗ 错误: 无法找到起点附近的自由区域")
                return None
            print(f"✓ 调整起点: {start} -> {new_start}")
            start = new_start
        else:
            print(f"✓ 起点有效: {start}")

        # 检查并调整终点
        if not self.is_valid(goal[0], goal[1], binary_map):
            print(f"⚠ 终点 {goal} 位于障碍物上，正在寻找最近的自由区域...")
            new_goal = self.find_nearest_free(goal[0], goal[1], binary_map)
            if new_goal is None:
                print(f"✗ 错误: 无法找到终点附近的自由区域")
                return None
            print(f"✓ 调整终点: {goal} -> {new_goal}")
            goal = new_goal
        else:
            print(f"✓ 终点有效: {goal}")

        # 8方向移动
        movements = [
            (0, 1), (0, -1), (1, 0), (-1, 0),  # 上下左右
            (1, 1), (1, -1), (-1, 1), (-1, -1)  # 对角线
        ]

        # 移动代价
        move_cost = [1, 1, 1, 1, 1.414, 1.414, 1.414, 1.414]

        # 优先队列: (f_score, g_score, x, y)
        open_set = []
        heapq.heappush(open_set, (0, 0, start[0], start[1]))

        # 记录来源
        came_from = {}
        came_from[start] = None

        # g_score: 从起点到当前点的实际代价
        g_score = {}
        g_score[start] = 0

        # f_score: g_score + heuristic
        f_score = {}
        f_score[start] = self.heuristic(start, goal)

        # 已访问节点
        closed_set = set()

        # 搜索次数
        iterations = 0
        max_iterations = self.width * self.height * 2

        print("开始搜索...")

        while open_set and iterations < max_iterations:
            iterations += 1

            # 取出f值最小的节点
            _, current_g, current_x, current_y = heapq.heappop(open_set)
            current = (current_x, current_y)

            # 到达目标
            if current == goal:
                print(f"✓ 找到路径! 搜索次数: {iterations}")
                # 重建路径
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.reverse()
                return path

            # 已访问则跳过
            if current in closed_set:
                continue

            closed_set.add(current)

            # 扩展邻居节点
            for i, (dx, dy) in enumerate(movements):
                neighbor = (current[0] + dx, current[1] + dy)

                # 检查有效性
                if not self.is_valid(neighbor[0], neighbor[1], binary_map):
                    continue

                if neighbor in closed_set:
                    continue

                # 计算新的g_score
                tentative_g = g_score[current] + move_cost[i]

                # 如果找到更短的路径
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], tentative_g,
                                              neighbor[0], neighbor[1]))

        print(f"✗ 未找到路径! 搜索次数: {iterations}")
        return None

    def visualize_path(self, binary_map: np.ndarray, path: List[Tuple[int, int]],
                      start: Tuple[int, int], goal: Tuple[int, int],
                      output_path: str = "path_result.png"):
        """可视化路径"""
        print("\n" + "="*60)
        print("步骤4: 可视化路径")
        print("="*60)

        # 创建彩色图像
        vis_map = cv2.cvtColor(binary_map, cv2.COLOR_GRAY2BGR)

        # 绘制路径 (绿色)
        for i, (x, y) in enumerate(path):
            cv2.circle(vis_map, (x, y), 1, (0, 255, 0), -1)

        # 绘制路径线
        for i in range(len(path) - 1):
            cv2.line(vis_map, path[i], path[i+1], (0, 255, 0), 2)

        # 绘制起点 (蓝色)
        cv2.circle(vis_map, start, 5, (255, 0, 0), -1)
        cv2.putText(vis_map, "START", (start[0] - 20, start[1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

        # 绘制终点 (红色)
        cv2.circle(vis_map, goal, 5, (0, 0, 255), -1)
        cv2.putText(vis_map, "GOAL", (goal[0] - 20, goal[1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        # 保存结果
        cv2.imwrite(output_path, vis_map)
        print(f"✓ 路径可视化已保存: {output_path}")

        return vis_map

    def export_path(self, path: List[Tuple[int, int]], output_file: str):
        """导出路径到文件"""
        print("\n" + "="*60)
        print("步骤5: 导出路径")
        print("="*60)

        with open(output_file, 'w') as f:
            f.write("# 路径规划结果\n")
            f.write(f"# 总步数: {len(path)}\n")
            f.write(f"# 地图分辨率: {self.map_config.resolution} m/pixel\n")
            f.write("# 格式: 像素坐标(x,y) -> 世界坐标(world_x, world_y)\n\n")

            for i, (px, py) in enumerate(path):
                world_x, world_y = self.map_to_world(px, py)
                f.write(f"{i+1:4d}. 像素({px:4d}, {py:4d}) -> 世界({world_x:8.3f}, {world_y:8.3f})\n")

        print(f"✓ 路径已导出到: {output_file}")

        # 计算路径总长度
        total_length = 0.0
        for i in range(len(path) - 1):
            dx = (path[i+1][0] - path[i][0]) * self.map_config.resolution
            dy = (path[i+1][1] - path[i][1]) * self.map_config.resolution
            total_length += np.sqrt(dx*dx + dy*dy)

        print(f"✓ 路径总长度: {total_length:.2f} 米 ({len(path)} 个路径点)")


def main():
    """主函数"""
    print("\n" + "="*60)
    print("   ROS 栅格地图路径规划工具")
    print("="*60)

    # 地图路径
    map_dir = r"E:\ai\agent\workspace\map"
    pgm_file = os.path.join(map_dir, "map.pgm")
    yaml_file = os.path.join(map_dir, "map.yaml")

    # 创建规划器
    planner = PathPlanner(pgm_file, yaml_file)

    # 加载地图
    if not planner.load_map():
        print("\n程序退出")
        return

    # 预处理地图
    binary_map = planner.preprocess_map()

    # 自动查找地图中的自由区域作为起点和终点
    print("\n" + "="*60)
    print("自动查找自由区域...")
    print("="*60)

    free_centers = planner.find_free_region_centers(binary_map, num_regions=10)

    if len(free_centers) < 2:
        print("✗ 错误: 地图中自由区域太少，无法进行路径规划")
        print("\n程序退出")
        return

    print(f"✓ 找到 {len(free_centers)} 个自由区域中心点")

    # 选择两个距离较远的点作为起点和终点
    # 使用第一个和最后一个点
    start_pixel = free_centers[0]
    goal_pixel = free_centers[-1]

    # 转换为世界坐标
    start_world = planner.map_to_world(start_pixel[0], start_pixel[1])
    goal_world = planner.map_to_world(goal_pixel[0], goal_pixel[1])

    print(f"\n起点(世界坐标): ({start_world[0]:.2f}, {start_world[1]:.2f})")
    print(f"终点(世界坐标): ({goal_world[0]:.2f}, {goal_world[1]:.2f})")

    # 执行路径规划
    path = planner.a_star(start_pixel, goal_pixel, binary_map)

    if path:
        # 可视化路径
        output_img = os.path.join(map_dir, "path_result.png")
        planner.visualize_path(binary_map, path, start_pixel, goal_pixel, output_img)

        # 导出路径
        output_txt = os.path.join(map_dir, "path_output.txt")
        planner.export_path(path, output_txt)

        print("\n" + "="*60)
        print("✓ 路径规划完成!")
        print("="*60)
        print(f"✓ 结果图像: {output_img}")
        print(f"✓ 路径文件: {output_txt}")
    else:
        print("\n✗ 路径规划失败")


if __name__ == "__main__":
    main()