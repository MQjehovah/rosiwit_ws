#!/usr/bin/env python3
"""
ROS2 Coverage Planner 集成测试

直接测试项目的规划节点：
1. 启动规划节点
2. 发布地图到 /map 话题
3. 发布初始位置到 /initialpose
4. 触发 /plan_coverage 服务
5. 接收规划结果 /coverage_path
6. 输出覆盖率、转弯次数等统计
"""

import os
import sys
import time
import json
import yaml
import signal
import subprocess
import threading
import numpy as np
import cv2
from pathlib import Path
from dataclasses import dataclass
from typing import Optional, Dict, List

# ROS2 imports
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, PoseArray
from std_srvs.srv import Trigger
from visualization_msgs.msg import MarkerArray


@dataclass
class TestResult:
    """测试结果"""
    map_name: str
    coverage_rate: float
    path_length: float
    turn_count: int
    path_points: int
    processing_time: float
    success: bool
    message: str


class CoveragePlannerTestClient(Node):
    """测试客户端节点"""

    def __init__(self):
        super().__init__('coverage_planner_test_client')

        self.callback_group = ReentrantCallbackGroup()

        # 发布者
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

        # 使用与规划节点匹配的QoS设置（reliable传输）
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        self.map_publisher = self.create_publisher(
            OccupancyGrid, '/map', qos_profile)

        self.initial_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)

        # 订阅者
        self.path_subscription = self.create_subscription(
            PoseArray, '/coverage_path', self.path_callback, 10,
            callback_group=self.callback_group)

        self.marker_subscription = self.create_subscription(
            MarkerArray, '/coverage_markers', self.marker_callback, 10,
            callback_group=self.callback_group)

        # 服务客户端
        self.plan_client = self.create_client(
            Trigger, '/plan_coverage', callback_group=self.callback_group)

        # 状态变量
        self.map_received_by_planner = False
        self.path_received = False
        self.coverage_path = None
        self.markers = None
        self.planning_result = None

        self.get_logger().info('Coverage planner test client initialized')

    def path_callback(self, msg: PoseArray):
        """接收规划路径"""
        self.coverage_path = msg
        self.path_received = True
        self.get_logger().info(
            f'Received coverage path with {len(msg.poses)} points')

    def marker_callback(self, msg: MarkerArray):
        """接收可视化标记"""
        self.markers = msg
        self.get_logger().info(f'Received {len(msg.markers)} visualization markers')

    def load_map_from_pgm(self, pgm_path: str, yaml_path: str) -> OccupancyGrid:
        """从PGM和YAML文件加载地图"""
        # 加载PGM图像
        img = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            raise ValueError(f'Failed to load map image: {pgm_path}')

        # 加载YAML元数据
        with open(yaml_path, 'r') as f:
            map_info = yaml.safe_load(f)

        # 创建OccupancyGrid消息
        grid = OccupancyGrid()

        # 头部信息
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = 'map'

        # 地图元数据
        grid.info.resolution = map_info.get('resolution', 0.05)
        grid.info.width = img.shape[1]
        grid.info.height = img.shape[0]

        # 原点位置
        origin = map_info.get('origin', [0.0, 0.0, 0.0])
        grid.info.origin.position.x = origin[0]
        grid.info.origin.position.y = origin[1]
        grid.info.origin.position.z = 0.0
        grid.info.origin.orientation.w = 1.0

        # 转换图像数据
        # PGM格式: 白色(255) = 可通行, 黑色(0) = 障碍, 灰色(205) = 未知
        grid.data = []
        for row in img:
            for pixel in row:
                if pixel == 255:  # 可通行
                    grid.data.append(0)
                elif pixel == 0:  # 障碍
                    grid.data.append(100)
                else:  # 未知（处理为可通行）
                    grid.data.append(0)

        self.get_logger().info(
            f'Loaded map: {pgm_path}, size: {grid.info.width}x{grid.info.height}, '
            f'resolution: {grid.info.resolution}m')

        return grid

    def publish_map(self, grid: OccupancyGrid):
        """发布地图"""
        # 持续发布确保被接收（DDS可靠传输需要时间建立连接）
        publish_count = 0
        for i in range(20):  # 持续发布20次，每次间隔0.5秒
            grid.header.stamp = self.get_clock().now().to_msg()
            self.map_publisher.publish(grid)
            publish_count += 1

            # 每5次检查是否已接收
            if i % 5 == 0:
                rclpy.spin_once(self, timeout_sec=0.1)

            time.sleep(0.5)

        self.get_logger().info(f'Map published {publish_count} times to /map topic')

    def publish_initial_pose(self, x: float, y: float):
        """发布初始位置"""
        pose = PoseWithCovarianceStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'

        pose.pose.pose.position.x = x
        pose.pose.pose.position.y = y
        pose.pose.pose.position.z = 0.0
        pose.pose.pose.orientation.w = 1.0

        # 协方差
        pose.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]

        self.initial_pose_publisher.publish(pose)
        self.get_logger().info(f'Initial pose published: ({x}, {y})')

    def trigger_planning(self, timeout: float = 30.0) -> bool:
        """触发规划服务"""
        if not self.plan_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Planning service not available')
            return False

        request = Trigger.Request()

        start_time = time.time()
        future = self.plan_client.call_async(request)

        # 等待服务响应
        while rclpy.ok() and time.time() - start_time < timeout:
            if future.done():
                try:
                    response = future.result()
                    self.planning_result = response
                    self.get_logger().info(f'Planning result: {response.message}')
                    return response.success
                except Exception as e:
                    self.get_logger().error(f'Service call failed: {e}')
                    return False
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().error('Planning service timeout')
        return False

    def calculate_statistics(self, grid: OccupancyGrid) -> Dict:
        """计算规划统计"""
        stats = {
            'coverage_rate': 0.0,
            'path_length': 0.0,
            'turn_count': 0,
            'path_points': 0,
            'free_cells': 0,
            'covered_cells': 0
        }

        # 统计可通行区域
        free_cells = sum(1 for val in grid.data if val == 0)
        stats['free_cells'] = free_cells

        # 如果收到路径
        if self.coverage_path:
            poses = self.coverage_path.poses
            stats['path_points'] = len(poses)

            # 计算路径长度
            total_length = 0.0
            for i in range(1, len(poses)):
                dx = poses[i].position.x - poses[i-1].position.x
                dy = poses[i].position.y - poses[i-1].position.y
                total_length += np.sqrt(dx*dx + dy*dy)
            stats['path_length'] = total_length

            # 估算覆盖率（基于路径点覆盖）
            resolution = grid.info.resolution
            coverage_radius = 0.3  # 机器人半径

            covered_mask = np.zeros((grid.info.height, grid.info.width), dtype=bool)

            for pose in poses:
                # 转换世界坐标到地图坐标
                mx = int((pose.position.x - grid.info.origin.position.x) / resolution)
                my = int((pose.position.y - grid.info.origin.position.y) / resolution)

                # 标记覆盖区域
                r = int(coverage_radius / resolution)
                for dy in range(-r, r+1):
                    for dx in range(-r, r+1):
                        y = my + dy
                        x = mx + dx
                        if 0 <= y < grid.info.height and 0 <= x < grid.info.width:
                            if dx*dx + dy*dy <= r*r:
                                # 检查是否是可通行区域
                                idx = y * grid.info.width + x
                                if grid.data[idx] == 0:
                                    covered_mask[y, x] = True

            covered_cells = np.sum(covered_mask)
            stats['covered_cells'] = int(covered_cells)
            stats['coverage_rate'] = covered_cells / free_cells * 100 if free_cells > 0 else 0

            # 计算转弯次数
            turn_count = 0
            for i in range(1, len(poses)-1):
                dx1 = poses[i].position.x - poses[i-1].position.x
                dy1 = poses[i].position.y - poses[i-1].position.y
                dx2 = poses[i+1].position.x - poses[i].position.x
                dy2 = poses[i+1].position.y - poses[i].position.y

                angle1 = np.arctan2(dy1, dx1)
                angle2 = np.arctan2(dy2, dx2)
                angle_change = abs(angle2 - angle1)

                if angle_change > np.pi:
                    angle_change = 2*np.pi - angle_change

                if angle_change > 0.1:  # 超过约5度算转弯
                    turn_count += 1

            stats['turn_count'] = turn_count

        return stats


def run_planner_node():
    """启动规划节点进程"""
    cmd = [
        'bash', '-c',
        'cd /mnt/e/ai/agent/workspace/projects/rosiwit_ws && '
        'source /opt/ros/humble/setup.bash && '
        'source install/setup.bash && '
        'ros2 run ros2_coverage_planner coverage_planner_node '
        '--ros-args --params-file '
        'src/rosiwit_coverage_planner/config/coverage_params.yaml'
    ]

    proc = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        preexec_fn=os.setsid
    )

    return proc


def test_single_map(client: CoveragePlannerTestClient,
                    map_name: str, map_dir: Path) -> TestResult:
    """测试单个地图"""
    sep = '='*60
    client.get_logger().info(f'\n{sep}')
    client.get_logger().info(f' Testing map: {map_name}')
    client.get_logger().info(sep)

    # 清除上次结果
    client.coverage_path = None
    client.path_received = False
    client.planning_result = None

    # 加载地图
    pgm_path = map_dir / f'{map_name}.pgm'
    yaml_path = map_dir / f'{map_name}.yaml'

    # 如果yaml不存在，创建默认的
    if not yaml_path.exists():
        yaml_path = map_dir / 'map.yaml'

    if not pgm_path.exists():
        # 尝试PNG格式
        png_path = map_dir / f'{map_name}.png'
        if png_path.exists():
            # 转换PNG到PGM格式（简化处理）
            img = cv2.imread(str(png_path), cv2.IMREAD_GRAYSCALE)
            pgm_temp = map_dir / f'{map_name}_temp.pgm'
            cv2.imwrite(str(pgm_temp), img)
            pgm_path = pgm_temp
        else:
            client.get_logger().error(f'Map file not found: {map_name}')
            return TestResult(
                map_name=map_name,
                success=False,
                message='Map file not found'
            )

    try:
        grid = client.load_map_from_pgm(str(pgm_path), str(yaml_path))
    except Exception as e:
        client.get_logger().error(f'Failed to load map: {e}')
        return TestResult(
            map_name=map_name,
            success=False,
            message=f'Failed to load map: {e}'
        )

    # 发布地图
    client.publish_map(grid)
    time.sleep(2.0)  # 等待节点接收并处理地图

    # 检查规划器是否接收了地图
    for _ in range(10):
        rclpy.spin_once(client, timeout_sec=0.1)
        time.sleep(0.2)

    # 发布初始位置（地图中心）
    center_x = grid.info.origin.position.x + (grid.info.width * grid.info.resolution) / 2
    center_y = grid.info.origin.position.y + (grid.info.height * grid.info.resolution) / 2
    client.publish_initial_pose(center_x, center_y)

    time.sleep(1.0)

    # 触发规划
    start_time = time.time()
    success = client.trigger_planning(timeout=60.0)
    processing_time = time.time() - start_time

    # 等待路径
    for _ in range(50):
        if client.path_received:
            break
        rclpy.spin_once(client, timeout_sec=0.5)
        time.sleep(0.1)

    # 计算统计
    stats = client.calculate_statistics(grid)

    # 从服务响应获取结果
    coverage_rate = stats['coverage_rate']
    path_length = stats['path_length']
    turn_count = stats['turn_count']

    if client.planning_result and client.planning_result.success:
        # 解析服务响应中的统计信息
        msg = client.planning_result.message
        if 'Coverage rate:' in msg:
            parts = msg.split(',')
            for part in parts:
                if 'Coverage rate:' in part:
                    coverage_rate = float(part.split(':')[1].strip().replace('%', ''))
                if 'Path length:' in part:
                    path_length = float(part.split(':')[1].strip().replace('m', ''))
                if 'Turn count:' in part:
                    turn_count = int(part.split(':')[1].strip())

    result = TestResult(
        map_name=map_name,
        coverage_rate=coverage_rate,
        path_length=path_length,
        turn_count=turn_count,
        path_points=stats['path_points'],
        processing_time=processing_time,
        success=success and client.path_received,
        message=client.planning_result.message if client.planning_result else 'No response'
    )

    client.get_logger().info(f'Test result:')
    client.get_logger().info(f'  Coverage rate: {result.coverage_rate:.2f}%')
    client.get_logger().info(f'  Path length: {result.path_length:.2f}m')
    client.get_logger().info(f'  Turn count: {result.turn_count}')
    client.get_logger().info(f'  Path points: {result.path_points}')
    client.get_logger().info(f'  Processing time: {result.processing_time:.2f}s')
    client.get_logger().info(f'  Success: {result.success}')

    return result


def main():
    """主测试流程"""
    print('\n' + '='*70)
    print(' ROS2 Coverage Planner Integration Test')
    print('='*70 + '\n')

    map_dir = Path('/mnt/e/ai/agent/workspace/projects/rosiwit_ws/src/rosiwit_coverage_planner/map')

    # 0. 清理残留节点
    print('[步骤0] Cleaning up residual nodes...')
    import subprocess as sp
    try:
        # 只杀掉ROS2节点进程（通过进程组）
        result = sp.run(
            ['bash', '-c', 'source /opt/ros/humble/setup.bash && ros2 node list'],
            capture_output=True, text=True, timeout=5
        )

        # 检查是否有coverage_planner节点
        if 'coverage_planner' in result.stdout:
            print('[清理] Found residual coverage_planner nodes, killing...')
            # 使用更精确的命令
            sp.run(
                ['bash', '-c', 'pkill -f "ros2 run ros2_coverage_planner"'],
                capture_output=True
            )
            time.sleep(2.0)
        else:
            print('[检查] No residual coverage_planner nodes found')
    except Exception as e:
        print(f'[警告] Cleanup check failed: {e}')

    # 1. 先启动规划节点
    print('[步骤1] Starting coverage planner node...')
    planner_proc = run_planner_node()

    # 2. 等待节点启动并检测就绪状态
    print('[步骤2] Waiting for planner node to initialize...')

    import subprocess as sp
    node_ready = False
    for i in range(30):  # 等待最多30秒
        try:
            result = sp.run(
                ['bash', '-c', 'source /opt/ros/humble/setup.bash && ros2 node list'],
                capture_output=True, text=True, timeout=5
            )
            if 'coverage_planner_node' in result.stdout:
                print(f'[就绪] Planner node detected after {i+1} seconds')
                node_ready = True
                break
        except:
            pass
        time.sleep(1)

    if not node_ready:
        print('[失败] Planner node not detected after 30 seconds')
        if planner_proc.poll() is None:
            os.killpg(os.getpgid(planner_proc.pid), signal.SIGTERM)
        return

    # 3. 额外等待确保订阅建立
    time.sleep(3.0)

    # 4. 检查服务是否可用
    print('[步骤3] Checking planning service...')
    try:
        result = sp.run(
            ['bash', '-c', 'source /opt/ros/humble/setup.bash && ros2 service list'],
            capture_output=True, text=True, timeout=5
        )
        print(f'[服务] Available services: {result.stdout}')
    except Exception as e:
        print(f'[警告] Could not list services: {e}')

    # 5. 初始化ROS2客户端（在节点启动后）
    print('[步骤4] Initializing test client...')
    rclpy.init()

    try:
        client = CoveragePlannerTestClient()
        executor = MultiThreadedExecutor()
        executor.add_node(client)

        # 在线程中运行executor
        spin_thread = threading.Thread(
            target=lambda: executor.spin(),
            daemon=True
        )
        spin_thread.start()

        # 等待客户端连接到节点
        time.sleep(2.0)

        # 测试地图列表
        test_maps = ['map']  # 使用map.pgm

        results = []

        for map_name in test_maps[:3]:  # 最多测试3个地图
            result = test_single_map(client, map_name, map_dir)
            results.append(result)
            time.sleep(2.0)

        # 生成报告
        print('\n' + '='*70)
        print(' Test Results Summary')
        print('='*70)

        for result in results:
            status = '✅' if result.success else '❌'
            print(f'{status} {result.map_name}:')
            print(f'   Coverage: {result.coverage_rate:.2f}%')
            print(f'   Path Length: {result.path_length:.2f}m')
            print(f'   Turn Count: {result.turn_count}')
            print(f'   Processing Time: {result.processing_time:.2f}s')
            print(f'   Message: {result.message[:50]}...' if len(result.message) > 50 else f'   Message: {result.message}')

        # 保存JSON结果
        results_json = []
        for r in results:
            results_json.append({
                'map_name': r.map_name,
                'success': r.success,
                'coverage_rate': r.coverage_rate,
                'path_length': r.path_length,
                'turn_count': r.turn_count,
                'path_points': r.path_points,
                'processing_time': r.processing_time,
                'message': r.message
            })

        output_path = map_dir / 'ros2_integration_test_results.json'
        with open(output_path, 'w') as f:
            json.dump(results_json, f, indent=2)

        print(f'\n[保存] Results saved to: {output_path}')

    finally:
        # 清理
        executor.shutdown()
        rclpy.shutdown()

        # 停止规划节点
        if planner_proc.poll() is None:
            print('\n[清理] Stopping planner node...')
            os.killpg(os.getpgid(planner_proc.pid), signal.SIGTERM)
            planner_proc.wait(timeout=5)

        print('\n' + '='*70)
        print(' Test Completed')
        print('='*70)


if __name__ == '__main__':
    main()