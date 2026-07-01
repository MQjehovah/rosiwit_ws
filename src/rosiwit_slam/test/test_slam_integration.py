#!/usr/bin/env python3
"""
FAST-LIO2 SLAM 集成测试脚本（简化版）
快速验证节点启动和接口连接
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import subprocess
import time
import signal
import sys

class SLAMIntegrationTest(Node):
    def __init__(self):
        super().__init__('slam_integration_test')
        
        # 统计
        self.node_started = False
        self.topics_available = []
        
        # 启动节点
        self.start_slam_node()
        
        # 检查接口
        self.check_interfaces()
        
        self.get_logger().info("="*60)
        self.get_logger().info("集成测试完成")
        self.print_results()
        
    def start_slam_node(self):
        """启动SLAM节点"""
        self.get_logger().info("="*60)
        self.get_logger().info("[1] 启动SLAM节点...")
        
        try:
            # 启动节点
            self.node_process = subprocess.Popen(
                [
                    "bash", "-c",
                    "source /opt/ros/humble/setup.bash && "
                    "cd /home/jmq/agent/workspace/project/fast_lio2_slam && "
                    "source install/setup.bash && "
                    "ros2 run fast_lio2_slam fast_lio2_slam --ros-args -p use_sim_time:=true"
                ],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=lambda: signal.signal(signal.SIGINT, signal.SIG_IGN)
            )
            
            # 等待启动
            time.sleep(5)
            
            # 检查是否存活
            if self.node_process.poll() is None:
                self.node_started = True
                self.get_logger().info("✅ SLAM节点启动成功")
                self.get_logger().info(f"   PID: {self.node_process.pid}")
            else:
                self.get_logger().error("❌ SLAM节点启动失败")
                
        except Exception as e:
            self.get_logger().error(f"❌ 启动异常: {e}")
            
    def check_interfaces(self):
        """检查ROS2接口"""
        self.get_logger().info("="*60)
        self.get_logger().info("[2] 检查ROS2接口...")
        
        try:
            # 获取话题列表
            result = subprocess.run(
                ["bash", "-c",
                 "source /opt/ros/humble/setup.bash && ros2 topic list"],
                capture_output=True,
                text=True,
                timeout=10
            )
            
            topics = result.stdout.strip().split('\n')
            expected_topics = [
                '/lidar_points',
                '/imu/data',
                '/odom_estimated',
                '/path_estimated',
                '/cloud_map'
            ]
            
            for topic in expected_topics:
                if topic in topics:
                    self.topics_available.append(topic)
                    self.get_logger().info(f"✅ {topic}")
                else:
                    self.get_logger().warning(f"⚠️  {topic} 未找到")
                    
            # 获取服务列表
            result_srv = subprocess.run(
                ["bash", "-c",
                 "source /opt/ros/humble/setup.bash && ros2 service list"],
                capture_output=True,
                text=True,
                timeout=10
            )
            
            services = result_srv.stdout.strip().split('\n')
            expected_services = ['/save_map', '/save_pcd']
            
            self.get_logger().info("")
            self.get_logger().info("服务:")
            for srv in expected_services:
                if srv in services:
                    self.get_logger().info(f"✅ {srv}")
                else:
                    self.get_logger().warning(f"⚠️  {srv} 未找到")
                    
        except subprocess.TimeoutExpired:
            self.get_logger().error("❌ 检查接口超时")
        except Exception as e:
            self.get_logger().error(f"❌ 检查异常: {e}")
            
    def print_results(self):
        """打印测试结果"""
        self.get_logger().info("="*60)
        self.get_logger().info("测试结果汇总:")
        
        if self.node_started:
            self.get_logger().info("✅ 节点启动: PASS")
        else:
            self.get_logger().info("❌ 节点启动: FAIL")
            
        if len(self.topics_available) >= 3:
            self.get_logger().info(f"✅ 话题接口: PASS ({len(self.topics_available)}/5)")
        else:
            self.get_logger().info(f"⚠️  话题接口: PARTIAL ({len(self.topics_available)}/5)")
            
        self.get_logger().info("="*60)
        
        # 清理
        if hasattr(self, 'node_process') and self.node_process.poll() is None:
            self.get_logger().info("清理节点进程...")
            self.node_process.terminate()
            self.node_process.wait(timeout=5)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        test_node = SLAMIntegrationTest()
        # 不需要spin，因为测试在构造函数中完成
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()