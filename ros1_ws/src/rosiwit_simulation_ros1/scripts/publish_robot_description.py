#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""把 /robot_description 参数以 latched 话题发出来，给 RosView / RViz 之类的外挂可视化用。

melodic 的 robot_state_publisher 只设置参数、不发同名话题（新版才会），
所以 RosView 的 3D 面板 auto-detect 不到 URDF。这里补一个 30 行的小节点。
"""
import rospy
from std_msgs.msg import String


class RobotDescriptionPublisher(object):
    def __init__(self):
        self.topic = rospy.get_param('~topic', '/robot_description')
        self.param = rospy.get_param('~param', '/robot_description')
        self.rate = float(rospy.get_param('~republish_rate', 0.0))
        self.pub = rospy.Publisher(self.topic, String, queue_size=1, latch=True)

    def run(self):
        desc = rospy.get_param(self.param, '')
        if not desc:
            rospy.logwarn('[robot_description_publisher] %s 为空，跳过', self.param)
            return
        msg = String(data=desc)
        self.pub.publish(msg)
        rospy.loginfo('[robot_description_publisher] 已发布 %s（%d 字节，latched）',
                      self.topic, len(desc))
        if self.rate > 0:
            r = rospy.Rate(self.rate)
            while not rospy.is_shutdown():
                self.pub.publish(msg)
                r.sleep()
        else:
            rospy.spin()


if __name__ == '__main__':
    rospy.init_node('robot_description_publisher')
    RobotDescriptionPublisher().run()
