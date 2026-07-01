#!/usr/bin/env python3
"""
机器人运动脚本 - 用于 Gazebo 仿真建图
让机器人在仿真环境中按8字形轨迹运动，以便 SLAM 建图覆盖更多区域。
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import time


class Figure8Driver(Node):
    """8字形轨迹驱动器，用于建图"""

    def __init__(self):
        super().__init__('figure8_driver')

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 参数
        self.declare_parameter('linear_speed', 0.3)
        self.declare_parameter('angular_speed', 0.5)
        self.declare_parameter('mode', 'figure8')  # figure8, circle, square, explore
        self.declare_parameter('duration', 60.0)   # 运动总时长(秒)

        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.mode = self.get_parameter('mode').value
        self.duration = self.get_parameter('duration').value

        self.start_time = time.time()
        self.get_logger().info(
            f'启动运动模式: {self.mode}, '
            f'线速度={self.linear_speed}, 角速度={self.angular_speed}, '
            f'时长={self.duration}s'
        )

        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        elapsed = time.time() - self.start_time

        if elapsed > self.duration:
            self.get_logger().info('运动完成，停止机器人')
            twist = Twist()
            self.cmd_vel_pub.publish(twist)
            return

        twist = Twist()

        if self.mode == 'figure8':
            # 8字形轨迹
            period = 10.0  # 一个完整8字周期
            t = elapsed % period
            if t < period / 2:
                # 前半圆（左转）
                twist.linear.x = self.linear_speed
                twist.angular.z = self.angular_speed
            else:
                # 后半圆（右转）
                twist.linear.x = self.linear_speed
                twist.angular.z = -self.angular_speed

        elif self.mode == 'circle':
            # 圆形轨迹
            twist.linear.x = self.linear_speed
            twist.angular.z = self.angular_speed

        elif self.mode == 'square':
            # 方形轨迹
            side_duration = 5.0  # 直线行驶时间
            turn_duration = 2.0  # 转弯时间
            cycle = side_duration + turn_duration
            t = elapsed % cycle
            if t < side_duration:
                twist.linear.x = self.linear_speed
                twist.angular.z = 0.0
            else:
                twist.linear.x = 0.0
                twist.angular.z = self.angular_speed * 1.5

        elif self.mode == 'explore':
            # 探索模式：缓慢前进，周期性改变方向
            period = 8.0
            t = elapsed % period
            twist.linear.x = self.linear_speed
            # 正弦转向
            twist.angular.z = self.angular_speed * math.sin(2 * math.pi * t / period)

        else:
            twist.linear.x = self.linear_speed
            twist.angular.z = 0.0

        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = Figure8Driver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # 停止机器人
        twist = Twist()
        node.cmd_vel_pub.publish(twist)
        node.get_logger().info('收到中断信号，停止机器人')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
