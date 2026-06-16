#!/usr/bin/env python3
"""
轻量级传感器数据模拟器 - 替代 Gazebo
模拟机器人在矩形房间中移动，生成 3D 点云和 IMU 数据
直接驱动 rosiwit_slam (FAST-LIO2) 建图
"""

import math
import time
import struct
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField, Imu
from geometry_msgs.msg import Vector3, Quaternion
from std_msgs.msg import Header
from builtin_interfaces.msg import Time


def create_pointcloud2(points, stamp, frame_id='velodyne'):
    """将 Nx3 numpy 数组转换为 PointCloud2 消息"""
    msg = PointCloud2()
    msg.header = Header()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = 1
    msg.width = len(points)
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 16  # 4 fields x 4 bytes
    msg.row_step = msg.point_step * msg.width
    msg.is_dense = True

    # 构建二进制数据
    buf = bytearray()
    for p in points:
        buf.extend(struct.pack('ffff', p[0], p[1], p[2], 50.0))
    msg.data = bytes(buf)
    return msg


def euler_to_quaternion(roll, pitch, yaw):
    """欧拉角转四元数"""
    cr = math.cos(roll / 2)
    sr = math.sin(roll / 2)
    cp = math.cos(pitch / 2)
    sp = math.sin(pitch / 2)
    cy = math.cos(yaw / 2)
    sy = math.sin(yaw / 2)
    return Quaternion(
        w=cr * cp * cy + sr * sp * sy,
        x=sr * cp * cy - cr * sp * sy,
        y=cr * sp * cy + sr * cp * sy,
        z=cr * cp * sy - sr * sp * cy
    )


class SimulatedSensorNode(Node):
    """模拟传感器节点：模拟 3D LiDAR + IMU"""

    def __init__(self):
        super().__init__('simulated_sensor')

        # 参数
        self.declare_parameter('room_size_x', 10.0)
        self.declare_parameter('room_size_y', 8.0)
        self.declare_parameter('room_height', 3.0)
        self.declare_parameter('scan_rate', 10.0)  # Hz
        self.declare_parameter('linear_speed', 0.5)
        self.declare_parameter('angular_speed', 0.3)

        self.room_x = self.get_parameter('room_size_x').value
        self.room_y = self.get_parameter('room_size_y').value
        self.room_z = self.get_parameter('room_height').value
        self.scan_rate = self.get_parameter('scan_rate').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value

        # 发布者
        # LiDAR: BEST_EFFORT (匹配 SLAM SensorDataQoS)
        # IMU: RELIABLE (解决 DDS 发现兼容性问题，SLAM 用 BEST_EFFORT 订阅兼容 RELIABLE 发布)
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        lidar_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        imu_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=200
        )
        self.lidar_pub = self.create_publisher(PointCloud2, '/velodyne_points', lidar_qos)
        self.imu_pub = self.create_publisher(Imu, '/imu', imu_qos)

        # 机器人状态
        self.robot_x = 1.0
        self.robot_y = 1.0
        self.robot_yaw = 0.0
        self.t = 0.0
        self.dt = 0.1

        # 定时器
        self.imu_timer = self.create_timer(0.02, self.imu_callback)   # 50 Hz
        self.lidar_timer = self.create_timer(1.0 / self.scan_rate, self.lidar_callback)  # 10 Hz
        self.move_timer = self.create_timer(self.dt, self.move_callback)  # 10 Hz

        # VLP-16 参数
        self.num_rings = 16
        self.vlp_fov_up = 15.0   # degrees
        self.vlp_fov_down = -15.0
        self.vlp_range = 100.0
        self.num_points_per_ring = 1800

        self.get_logger().info(
            f'Simulated sensor started: room={self.room_x}x{self.room_y}x{self.room_z}, '
            f'scan_rate={self.scan_rate}Hz'
        )

        # 路径点列表（8字形轨迹）
        self.waypoints = self._generate_figure8_waypoints()
        self.wp_index = 0

    def _generate_figure8_waypoints(self):
        """生成 8 字形轨迹路径点"""
        waypoints = []
        cx, cy = self.room_x / 2, self.room_y / 2
        r = min(self.room_x, self.room_y) * 0.3
        for i in range(200):
            t = i * 0.05
            x = cx + r * math.sin(t)
            y = cy + r * math.sin(t) * math.cos(t)
            # 保持在房间内
            x = max(0.5, min(self.room_x - 0.5, x))
            y = max(0.5, min(self.room_y - 0.5, y))
            waypoints.append((x, y))
        return waypoints

    def move_callback(self):
        """更新机器人位置"""
        self.t += self.dt

        # 沿8字形轨迹移动
        if self.wp_index < len(self.waypoints):
            tx, ty = self.waypoints[self.wp_index]
            dx = tx - self.robot_x
            dy = ty - self.robot_y
            dist = math.sqrt(dx * dx + dy * dy)
            if dist < 0.1:
                self.wp_index = (self.wp_index + 1) % len(self.waypoints)
                tx, ty = self.waypoints[self.wp_index]
                dx = tx - self.robot_x
                dy = ty - self.robot_y
                dist = math.sqrt(dx * dx + dy * dy)

            if dist > 0.01:
                target_yaw = math.atan2(dy, dx)
                # 平滑转向
                yaw_diff = target_yaw - self.robot_yaw
                while yaw_diff > math.pi:
                    yaw_diff -= 2 * math.pi
                while yaw_diff < -math.pi:
                    yaw_diff += 2 * math.pi
                self.robot_yaw += yaw_diff * 0.1

                step = min(self.linear_speed * self.dt, dist)
                self.robot_x += step * math.cos(self.robot_yaw)
                self.robot_y += step * math.sin(self.robot_yaw)

        # 保持在房间内
        self.robot_x = max(0.3, min(self.room_x - 0.3, self.robot_x))
        self.robot_y = max(0.3, min(self.room_y - 0.3, self.robot_y))

    def _generate_room_points(self):
        """生成模拟房间中的 3D 点云（四面墙 + 地面 + 天花板 + 一些障碍物）"""
        points = []

        # 机器人的旋转矩阵
        cos_yaw = math.cos(self.robot_yaw)
        sin_yaw = math.sin(self.robot_yaw)

        def transform(px, py, pz):
            """将世界坐标的点转换到机器人坐标系"""
            dx = px - self.robot_x
            dy = py - self.robot_y
            # 逆旋转
            rx = dx * cos_yaw + dy * sin_yaw
            ry = -dx * sin_yaw + dy * cos_yaw
            return [rx, ry, pz]

        # ---- 四面墙 ----
        wall_density = 0.15  # 每 0.15m 一个点

        # 左墙 (x=0)
        for y in np.arange(0, self.room_y, wall_density):
            for z in np.arange(0, self.room_z, wall_density * 2):
                points.append(transform(0, y, z))

        # 右墙 (x=room_x)
        for y in np.arange(0, self.room_y, wall_density):
            for z in np.arange(0, self.room_z, wall_density * 2):
                points.append(transform(self.room_x, y, z))

        # 前墙 (y=0)
        for x in np.arange(0, self.room_x, wall_density):
            for z in np.arange(0, self.room_z, wall_density * 2):
                points.append(transform(x, 0, z))

        # 后墙 (y=room_y)
        for x in np.arange(0, self.room_x, wall_density):
            for z in np.arange(0, self.room_z, wall_density * 2):
                points.append(transform(x, self.room_y, z))

        # ---- 地面 (z=0) ----
        for x in np.arange(0, self.room_x, wall_density * 2):
            for y in np.arange(0, self.room_y, wall_density * 2):
                points.append(transform(x, y, 0))

        # ---- 天花板 (z=room_z) ----
        for x in np.arange(0, self.room_x, wall_density * 3):
            for y in np.arange(0, self.room_y, wall_density * 3):
                points.append(transform(x, y, self.room_z))

        # ---- 内部障碍物（桌子、柱子等）----
        # 桌子1 (长方体 1x0.5x0.8 在 (3,3))
        for x in np.arange(2.5, 3.5, wall_density):
            for y in np.arange(2.5, 3.0, wall_density):
                for z in np.arange(0, 0.8, wall_density * 2):
                    points.append(transform(x, y, z))

        # 柱子1 (圆柱 在 (6,5))
        for angle in np.arange(0, 2 * math.pi, 0.1):
            for z in np.arange(0, self.room_z, wall_density):
                px = 6 + 0.3 * math.cos(angle)
                py = 5 + 0.3 * math.sin(angle)
                points.append(transform(px, py, z))

        # 柱子2 (圆柱 在 (2,6))
        for angle in np.arange(0, 2 * math.pi, 0.1):
            for z in np.arange(0, self.room_z, wall_density):
                px = 2 + 0.25 * math.cos(angle)
                py = 6 + 0.25 * math.sin(angle)
                points.append(transform(px, py, z))

        # 隔墙 (从 (5,0) 到 (5,4))
        for y in np.arange(0, 4, wall_density):
            for z in np.arange(0, 2.5, wall_density * 2):
                points.append(transform(5, y, z))

        # 只保留在传感器范围内的点
        filtered = []
        for p in points:
            r = math.sqrt(p[0]**2 + p[1]**2 + p[2]**2)
            if 0.5 < r < self.vlp_range:
                filtered.append(p)

        # 添加测量噪声（模拟真实 LiDAR）
        if filtered:
            pts = np.array(filtered)
            noise = np.random.normal(0, 0.02, pts.shape)  # 2cm 噪声
            pts += noise
            return pts.tolist()

        return filtered

    def lidar_callback(self):
        """发布模拟 3D 点云"""
        now = self.get_clock().now().to_msg()
        points = self._generate_room_points()

        if points:
            msg = create_pointcloud2(points, now, 'velodyne_link')
            self.lidar_pub.publish(msg)
            self.get_logger().debug(f'Published {len(points)} points')

    def imu_callback(self):
        """发布模拟 IMU 数据"""
        now = self.get_clock().now().to_msg()

        msg = Imu()
        msg.header = Header()
        msg.header.stamp = now
        msg.header.frame_id = 'imu_link'

        # 姿态
        q = euler_to_quaternion(0, 0, self.robot_yaw)
        msg.orientation = q
        msg.orientation_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]

        # 角速度
        angular_z = self.angular_speed * math.cos(self.t * 0.5)
        msg.angular_velocity = Vector3(
            x=0.001 * np.random.randn(),
            y=0.001 * np.random.randn(),
            z=angular_z + 0.001 * np.random.randn()
        )
        msg.angular_velocity_covariance = [0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001]

        # 线加速度
        msg.linear_acceleration = Vector3(
            x=self.linear_speed * 0.1 + 0.01 * np.random.randn(),
            y=angular_z * self.linear_speed + 0.01 * np.random.randn(),
            z=9.81 + 0.01 * np.random.randn()
        )
        msg.linear_acceleration_covariance = [0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1]

        self.imu_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimulatedSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
