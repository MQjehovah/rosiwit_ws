#!/usr/bin/env python3
"""
测试脚本：加载地图并触发覆盖路径规划
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped
import yaml
import cv2
import numpy as np
import time


class MapPublisher(Node):
    """地图发布节点"""
    
    def __init__(self):
        super().__init__('map_publisher')
        
        # 发布地图
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)
        
        # 发布初始位置
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        
        self.get_logger().info('Map Publisher Node 已启动')
    
    def publish_map(self, pgm_path, yaml_path):
        """加载并发布地图"""
        self.get_logger().info(f'加载地图: {pgm_path}')
        
        # 加载 YAML 配置
        with open(yaml_path, 'r') as f:
            config = yaml.safe_load(f)
        
        resolution = config.get('resolution', 0.05)
        origin = config.get('origin', [0, 0, 0])
        
        # 加载 PGM 图像
        img = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            self.get_logger().error(f'无法加载地图: {pgm_path}')
            return False
        
        height, width = img.shape
        
        # 创建 OccupancyGrid 消息
        map_msg = OccupancyGrid()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = 'map'
        
        map_msg.info.resolution = resolution
        map_msg.info.width = width
        map_msg.info.height = height
        map_msg.info.origin.position.x = origin[0]
        map_msg.info.origin.position.y = origin[1]
        map_msg.info.origin.position.z = 0.0
        map_msg.info.origin.orientation.w = 1.0
        
        # 转换图像数据为占用网格
        # PGM: 255=自由, 0=障碍, 127=未知
        # OccupancyGrid: 0=自由, 100=障碍, -1=未知
        occupied_thresh = config.get('occupied_thresh', 0.65)
        free_thresh = config.get('free_thresh', 0.196)
        negate = config.get('negate', 0)
        
        normalized = img.astype(float) / 255.0
        if negate:
            normalized = 1.0 - normalized
        
        # 转换为占用值
        occupancy = np.zeros((height, width), dtype=np.int8)
        
        # 自由区域 (< free_thresh)
        free_mask = normalized <= free_thresh
        occupancy[free_mask] = 0
        
        # 障碍区域 (> occupied_thresh)
        obstacle_mask = normalized >= occupied_thresh
        occupancy[obstacle_mask] = 100
        
        # 未知区域 (介于两者之间)
        unknown_mask = ~free_mask & ~obstacle_mask
        occupancy[unknown_mask] = -1
        
        # 翻转 Y 轴（PGM 原点在左下角）
        occupancy = np.flipud(occupancy)
        
        map_msg.data = occupancy.flatten().tolist()
        
        self.get_logger().info(f'地图尺寸: {width}x{height}, 分辨率: {resolution}m')
        self.get_logger().info(f'发布地图...')
        
        self.map_pub.publish(map_msg)
        
        return True
    
    def publish_initial_pose(self, x, y, yaw=0.0):
        """发布初始位置"""
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        
        pose_msg.pose.pose.position.x = x
        pose_msg.pose.pose.position.y = y
        pose_msg.pose.pose.position.z = 0.0
        
        # 设置朝向 (yaw)
        import math
        pose_msg.pose.pose.orientation.x = 0.0
        pose_msg.pose.pose.orientation.y = 0.0
        pose_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        self.get_logger().info(f'发布初始位置: ({x}, {y})')
        self.initial_pose_pub.publish(pose_msg)


def main():
    rclpy.init()
    
    node = MapPublisher()
    
    # 地图路径
    pgm_path = '/home/jmq/ros2_ws/src/ros2_coverage_planner/maps/map.pgm'
    yaml_path = '/home/jmq/ros2_ws/src/ros2_coverage_planner/maps/map.yaml'
    
    # 发布地图
    if node.publish_map(pgm_path, yaml_path):
        # 等待地图被接收
        time.sleep(1)
        
        # 发布初始位置（地图中心附近）
        # 地图原点 (-48.35, -37.87), 尺寸 1189x1117, 分辨率 0.05
        # 选择一个自由区域内的点
        node.publish_initial_pose(-20.0, -15.0)
        
        # 保持运行一段时间
        for _ in range(5):
            rclpy.spin_once(node, timeout_sec=0.1)
            time.sleep(0.1)
    
    node.get_logger().info('Map Publisher 完成')
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()