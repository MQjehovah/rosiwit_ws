#!/usr/bin/env python3
"""
一体化覆盖规划集成测试

在单个进程中测试完整流程：
1. 创建地图并发布
2. 规划节点订阅地图
3. 触发规划服务
4. 验证结果
"""

import os
import sys
import time
import numpy as np
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import Trigger


class IntegratedTest(Node):
    """一体化测试节点"""
    
    def __init__(self):
        super().__init__('integrated_test')
        
        # 创建订阅者（模拟规划节点的订阅）
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)
        
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose', self.pose_callback, 10)
        
        # 创建发布者
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        
        # 创建服务客户端
        self.cli = self.create_client(Trigger, '/plan_coverage')
        
        # 状态
        self.map_received = False
        self.pose_received = False
        self.received_map = None
        
        self.get_logger().info('Integrated test node initialized')
    
    def map_callback(self, msg):
        """地图回调"""
        self.map_received = True
        self.received_map = msg
        self.get_logger().info(f'Map received: {msg.info.width}x{msg.info.height}')
    
    def pose_callback(self, msg):
        """位姿回调"""
        self.pose_received = True
        self.get_logger().info(f'Pose received: ({msg.pose.pose.position.x:.2f}, {msg.pose.pose.position.y:.2f})')
    
    def create_test_map(self):
        """创建测试地图"""
        # 100x80栅格，分辨率0.05m
        # 5m x 4m的区域
        grid = OccupancyGrid()
        grid.header.frame_id = 'map'
        grid.header.stamp = self.get_clock().now().to_msg()
        
        grid.info.resolution = 0.05
        grid.info.width = 100
        grid.info.height = 80
        grid.info.origin.position.x = 0.0
        grid.info.origin.position.y = 0.0
        grid.info.origin.orientation.w = 1.0
        
        # 创建障碍物布局
        # 外围障碍物
        data = np.zeros((80, 100), dtype=np.int8)
        
        # 边界障碍物
        data[0, :] = 100
        data[79, :] = 100
        data[:, 0] = 100
        data[:, 99] = 100
        
        # 内部障碍物（L形）
        data[20:25, 30:50] = 100
        data[20:40, 30:35] = 100
        
        # 另一个障碍物块
        data[50:70, 60:65] = 100
        data[50:55, 60:80] = 100
        
        grid.data = data.flatten().tolist()
        
        self.get_logger().info(f'Created test map: {grid.info.width}x{grid.info.height}')
        return grid
    
    def publish_map(self, grid):
        """发布地图"""
        for i in range(5):
            grid.header.stamp = self.get_clock().now().to_msg()
            self.map_pub.publish(grid)
            self.get_logger().info(f'Published map iteration {i+1}')
            time.sleep(0.2)
    
    def publish_pose(self, grid):
        """发布初始位姿"""
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        
        # 地图中心
        pose.pose.pose.position.x = grid.info.width * grid.info.resolution / 2
        pose.pose.pose.position.y = grid.info.height * grid.info.resolution / 2
        pose.pose.pose.orientation.w = 1.0
        
        self.pose_pub.publish(pose)
        self.get_logger().info(f'Published initial pose')
    
    def run_test(self):
        """运行测试"""
        # 1. 创建测试地图
        grid = self.create_test_map()
        
        # 2. 发布地图
        self.publish_map(grid)
        
        # 等待地图被接收
        time.sleep(1)
        
        # 3. 发布初始位姿
        self.publish_pose(grid)
        
        # 等待位姿被接收
        time.sleep(0.5)
        
        # 4. 调用规划服务
        self.get_logger().info('Calling plan_coverage service...')
        
        while not self.cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('Waiting for service /plan_coverage...')
            if self.cli.service_is_ready():
                break
        
        req = Trigger.Request()
        future = self.cli.call_async(req)
        
        # 等待结果
        start_time = time.time()
        while time.time() - start_time < 30.0:
            rclpy.spin_once(self, timeout_sec=0.1)
            if future.done():
                break
        
        if future.done():
            try:
                result = future.result()
                self.get_logger().info(f'Service result: success={result.success}')
                self.get_logger().info(f'Message: {result.message}')
                return result.success
            except Exception as e:
                self.get_logger().error(f'Service call failed: {e}')
                return False
        else:
            self.get_logger().error('Service call timeout')
            return False


def main():
    rclpy.init()
    
    try:
        node = IntegratedTest()
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        
        # 在单独线程中spin
        import threading
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()
        
        # 运行测试
        success = node.run_test()
        
        if success:
            node.get_logger().info('✅ Integrated test PASSED!')
        else:
            node.get_logger().error('❌ Integrated test FAILED!')
        
        # 清理
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        
        return 0 if success else 1
        
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())