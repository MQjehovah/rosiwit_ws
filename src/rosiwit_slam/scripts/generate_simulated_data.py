#!/usr/bin/env python3
"""
rosiwit_slam 模拟数据生成脚本
生成模拟LiDAR点云和IMU数据进行建图测试
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField, Imu
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Header
import numpy as np
import time
import math
from datetime import datetime


class SimulatedDataPublisher(Node):
    """发布模拟LiDAR和IMU数据的节点"""
    
    def __init__(self):
        super().__init__('simulated_data_publisher')
        
        # QoS配置
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # 创建发布者
        self.lidar_pub = self.create_publisher(
            PointCloud2, '/lidar_points', qos_profile)
        self.imu_pub = self.create_publisher(
            Imu, '/imu/data', qos_profile)
        
        # 模拟参数
        self.lidar_rate = 10.0  # 10Hz
        self.imu_rate = 100.0   # 100Hz
        self.num_points = 1000  # 每帧点数
        
        # 轨迹参数（模拟运动）
        self.linear_velocity = 0.5  # m/s
        self.angular_velocity = 0.1  # rad/s
        self.current_time = 0.0
        
        # 定时器
        self.lidar_timer = self.create_timer(
            1.0 / self.lidar_rate, self.publish_lidar)
        self.imu_timer = self.create_timer(
            1.0 / self.imu_rate, self.publish_imu)
        
        self.get_logger().info('模拟数据发布节点已启动')
        self.get_logger().info(f'LiDAR频率: {self.lidar_rate}Hz, IMU频率: {self.imu_rate}Hz')
        self.get_logger().info(f'每帧点数: {self.num_points}')
        self.frame_count = 0
        
    def create_point_cloud(self, timestamp):
        """创建模拟点云数据"""
        # 点云字段定义（Ouster格式）
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='t', offset=16, datatype=PointField.FLOAT32, count=1),
            PointField(name='ring', offset=20, datatype=PointField.FLOAT32, count=1),
        ]
        
        # 生成模拟点云（模拟室内场景）
        points = []
        for i in range(self.num_points):
            # 模拟物体表面点（墙壁、地面等）
            theta = 2 * math.pi * i / self.num_points + self.current_time * self.angular_velocity
            
            # 墙壁点（距离3-10米）
            if i < self.num_points * 0.6:
                r = np.random.uniform(3.0, 10.0)
                x = r * math.cos(theta)
                y = r * math.sin(theta)
                z = np.random.uniform(-0.5, 2.0)  # 垂直分布
            # 地面点
            else:
                r = np.random.uniform(1.0, 8.0)
                x = r * math.cos(theta)
                y = r * math.sin(theta)
                z = -0.1 + np.random.normal(0, 0.05)
            
            intensity = np.random.uniform(0.5, 1.0)
            t = i / self.num_points * 0.1  # 时间戳偏移
            ring = int(i * 16 / self.num_points)  # 16线
            
            points.append([x, y, z, intensity, t, ring])
        
        # 创建PointCloud2消息
        header = Header()
        header.stamp = timestamp
        header.frame_id = 'lidar'
        
        # 将点转换为字节
        point_step = 24  # 6个字段，每个4字节
        data = bytearray()
        for p in points:
            for v in p:
                data.extend(np.array([v], dtype=np.float32).tobytes())
        
        cloud = PointCloud2()
        cloud.header = header
        cloud.height = 1
        cloud.width = self.num_points
        cloud.fields = fields
        cloud.is_bigendian = False
        cloud.point_step = point_step
        cloud.row_step = point_step * self.num_points
        cloud.is_dense = True
        cloud.data = bytes(data)
        
        return cloud
    
    def create_imu_data(self, timestamp):
        """创建模拟IMU数据"""
        imu = Imu()
        imu.header.stamp = timestamp
        imu.header.frame_id = 'imu'
        
        # 模拟加速度（包含运动加速度和重力）
        # 前向运动
        linear_acc_x = self.linear_velocity * 0.1 + np.random.normal(0, 0.01)
        # 重力（约9.81）
        linear_acc_z = 9.81 + np.random.normal(0, 0.05)
        linear_acc_y = np.random.normal(0, 0.01)
        
        imu.linear_acceleration.x = linear_acc_x
        imu.linear_acceleration.y = linear_acc_y
        imu.linear_acceleration.z = linear_acc_z
        
        # 模拟角速度（旋转）
        gyro_x = np.random.normal(0, 0.001)
        gyro_y = np.random.normal(0, 0.001)
        gyro_z = self.angular_velocity + np.random.normal(0, 0.005)
        
        imu.angular_velocity.x = gyro_x
        imu.angular_velocity.y = gyro_y
        imu.angular_velocity.z = gyro_z
        
        # 四元数（模拟旋转）
        angle = self.current_time * self.angular_velocity
        imu.orientation = Quaternion(
            x=0.0,
            y=0.0,
            z=math.sin(angle/2),
            w=math.cos(angle/2)
        )
        
        return imu
    
    def publish_lidar(self):
        """发布LiDAR数据"""
        timestamp = self.get_clock().now().to_msg()
        cloud = self.create_point_cloud(timestamp)
        self.lidar_pub.publish(cloud)
        self.frame_count += 1
        self.current_time += 1.0 / self.lidar_rate
        
        if self.frame_count % 50 == 0:
            self.get_logger().info(f'已发布 {self.frame_count} 帧点云')
    
    def publish_imu(self):
        """发布IMU数据"""
        timestamp = self.get_clock().now().to_msg()
        imu = self.create_imu_data(timestamp)
        self.imu_pub.publish(imu)
    
    def get_statistics(self):
        """获取统计信息"""
        return {
            'frames_published': self.frame_count,
            'duration': self.current_time
        }


def main(args=None):
    rclpy.init(args=args)
    
    node = SimulatedDataPublisher()
    
    try:
        print('\n========== 模拟数据生成开始 ==========')
        print('按Ctrl+C停止...')
        print('')
        
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        stats = node.get_statistics()
        print('\n========== 模拟数据统计 ==========')
        print(f'发布帧数: {stats["frames_published"]}')
        print(f'运行时长: {stats["duration"]: .2f} 秒')
        print('')
        
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()