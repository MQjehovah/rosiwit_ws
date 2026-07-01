#!/usr/bin/env python3
"""
地图发布脚本 - 向规划节点发送地图数据

使用方法：
1. 启动规划节点
2. 运行此脚本发布地图
3. 触发规划服务
"""

import os
import sys
import time
import yaml
import cv2
import numpy as np
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import Trigger


class MapPublisher(Node):
    def __init__(self):
        super().__init__('map_publisher')

        # 使用transient_local durability确保新订阅者也能接收到地图
        qos_profile = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.map_pub = self.create_publisher(
            OccupancyGrid, '/map', qos_profile)

        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)

        self.cli = self.create_client(Trigger, '/plan_coverage')

        self.get_logger().info('Map publisher initialized')

    def load_map(self, pgm_path, yaml_path):
        """加载ROS地图文件"""
        # 加载图像
        img = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            self.get_logger().error(f'Failed to load: {pgm_path}')
            return None

        # 加载YAML（忽略image路径，使用本地pgm）
        try:
            with open(yaml_path, 'r') as f:
                meta = yaml.safe_load(f)
        except Exception as e:
            self.get_logger().warn(f'Failed to load yaml: {e}, using defaults')
            meta = {}

        # 创建OccupancyGrid
        grid = OccupancyGrid()
        grid.header.frame_id = 'map'
        grid.header.stamp = self.get_clock().now().to_msg()

        grid.info.resolution = meta.get('resolution', 0.05)
        grid.info.width = img.shape[1]
        grid.info.height = img.shape[0]

        origin = meta.get('origin', [0.0, 0.0, 0.0])
        grid.info.origin.position.x = origin[0]
        grid.info.origin.position.y = origin[1]
        grid.info.origin.orientation.w = 1.0

        # 转换数据：255=可通行(0), 0=障碍(100), 其他=未知(-1)
        for row in img:
            for pixel in row:
                if pixel == 255:
                    grid.data.append(0)
                elif pixel == 0:
                    grid.data.append(100)
                else:
                    grid.data.append(0)  # 未知作为可通行

        self.get_logger().info(f'Map loaded: {grid.info.width}x{grid.info.height}')
        return grid

    def publish_map(self, grid):
        """持续发布地图"""
        # TRANSIENT_LOCAL durability: 新订阅者也会收到已发布的消息
        for i in range(10):
            grid.header.stamp = self.get_clock().now().to_msg()
            self.map_pub.publish(grid)
            self.get_logger().info(f'Published map iteration {i+1}')
            time.sleep(0.5)

    def publish_pose(self, grid):
        """发布初始位置"""
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()

        # 地图中心
        pose.pose.pose.position.x = grid.info.origin.position.x + grid.info.width * grid.info.resolution / 2
        pose.pose.pose.position.y = grid.info.origin.position.y + grid.info.height * grid.info.resolution / 2
        pose.pose.pose.orientation.w = 1.0

        self.pose_pub.publish(pose)
        self.get_logger().info(f'Initial pose: ({pose.pose.pose.position.x:.2f}, {pose.pose.pose.position.y:.2f})')

    def call_plan_service(self):
        """调用规划服务"""
        if not self.cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Service /plan_coverage not available')
            return None

        req = Trigger.Request()
        future = self.cli.call_async(req)

        # 等待结果
        rclpy.spin_until_future_complete(self, future, timeout_sec=60.0)

        if future.done():
            try:
                result = future.result()
                self.get_logger().info(f'Service result: {result.message}')
                return result
            except Exception as e:
                self.get_logger().error(f'Service call failed: {e}')
                return None
        else:
            self.get_logger().error('Service call timeout')
            return None


def main():
    map_dir = Path('/mnt/e/ai/agent/workspace/projects/rosiwit_ws/src/rosiwit_coverage_planner/map')
    pgm_path = map_dir / 'map.pgm'
    yaml_path = map_dir / 'map.yaml'

    rclpy.init()

    try:
        node = MapPublisher()

        # 加载地图
        grid = node.load_map(str(pgm_path), str(yaml_path))
        if grid is None:
            return

        # 发布地图（持续发布）
        node.publish_map(grid)
        time.sleep(2)

        # 发布初始位置
        node.publish_pose(grid)
        time.sleep(1)

        # 调用规划服务
        result = node.call_plan_service()

        if result and result.success:
            node.get_logger().info('✅ Coverage planning succeeded!')

            # 解析结果
            msg = result.message
            if 'Coverage rate:' in msg:
                parts = msg.split(',')
                for part in parts:
                    print(f'  {part}')
        else:
            node.get_logger().error('❌ Coverage planning failed')

    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()