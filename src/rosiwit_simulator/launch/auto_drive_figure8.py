#!/usr/bin/env python3
"""
自动驾驶脚本 - 8字形轨迹
发布 /cmd_vel 让机器人在仿真环境中走 8 字形，持续产生运动数据供 SLAM 建图
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class Figure8Driver(Node):
    def __init__(self):
        super().__init__('figure8_driver')

        self.declare_parameter('use_sim_time', True)
        self.declare_parameter('linear_speed', 0.3)
        self.declare_parameter('angular_speed', 0.5)

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.t = 0.0
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.get_logger().info(
            f'Figure-8 driver started: linear={self.linear_speed}, angular={self.angular_speed}'
        )

    def timer_callback(self):
        self.t += 0.1
        twist = Twist()
        # 8字形轨迹: 速度按正弦变化
        twist.linear.x = self.linear_speed
        twist.angular.z = self.angular_speed * math.sin(self.t * 0.5)
        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = Figure8Driver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 停止机器人
        twist = Twist()
        node.cmd_vel_pub.publish(twist)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
