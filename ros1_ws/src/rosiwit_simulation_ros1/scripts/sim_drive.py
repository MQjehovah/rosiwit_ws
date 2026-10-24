#!/usr/bin/env python
"""
ROS1 robot motion helper for Gazebo mapping tests.

Drives the robot in a figure-8 / circle / square / explore pattern so that
SLAM can cover more of the map. Mirrors the ROS2 scripts/sim_drive.py.
"""

import math
import time

import rospy
from geometry_msgs.msg import Twist


class Figure8Driver:
    def __init__(self):
        rospy.init_node('figure8_driver')

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.linear_speed = rospy.get_param('~linear_speed', 0.3)
        self.angular_speed = rospy.get_param('~angular_speed', 0.5)
        self.mode = rospy.get_param('~mode', 'figure8')
        self.duration = rospy.get_param('~duration', 60.0)
        self.rate = rospy.Rate(10.0)

        # Wall clock: before /clock arrives sim time is 0 and then jumps to the
        # world time (~460 s), so rospy.Time-based elapsed would exit instantly.
        self.start_time = time.time()
        rospy.loginfo(
            'motion mode=%s linear=%.2f angular=%.2f duration=%.1fs',
            self.mode, self.linear_speed, self.angular_speed, self.duration)

    def timer_callback(self, elapsed):
        twist = Twist()

        if self.mode == 'figure8':
            period = 10.0
            t = elapsed % period
            twist.linear.x = self.linear_speed
            twist.angular.z = self.angular_speed if t < period / 2 else -self.angular_speed
        elif self.mode == 'circle':
            twist.linear.x = self.linear_speed
            twist.angular.z = self.angular_speed
        elif self.mode == 'square':
            side_duration = 5.0
            turn_duration = 2.0
            cycle = side_duration + turn_duration
            t = elapsed % cycle
            if t < side_duration:
                twist.linear.x = self.linear_speed
                twist.angular.z = 0.0
            else:
                twist.linear.x = 0.0
                twist.angular.z = self.angular_speed * 1.5
        elif self.mode == 'explore':
            period = 8.0
            t = elapsed % period
            twist.linear.x = self.linear_speed
            twist.angular.z = self.angular_speed * math.sin(2.0 * math.pi * t / period)
        else:
            twist.linear.x = self.linear_speed
            twist.angular.z = 0.0

        return twist

    def run(self):
        while not rospy.is_shutdown():
            elapsed = time.time() - self.start_time
            if elapsed > self.duration:
                rospy.loginfo('motion finished, stopping robot')
                self.cmd_vel_pub.publish(Twist())
                return
            self.cmd_vel_pub.publish(self.timer_callback(elapsed))
            self.rate.sleep()


def main():
    driver = Figure8Driver()
    try:
        driver.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        driver.cmd_vel_pub.publish(Twist())


if __name__ == '__main__':
    main()
