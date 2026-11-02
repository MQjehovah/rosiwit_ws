#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Move the simulated persons along simple walking paths.

The models are visual-only and have gravity disabled; this node teleports them
with /gazebo/set_model_state so model_states follows and the GT detector /
tracker sees real motion (non-zero velocity).
"""
import math

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState


class PeopleMover(object):
    def __init__(self):
        names = rospy.get_param('~model_names', 'person_1,person_2,person_3')
        self.names = [n.strip() for n in names.split(',') if n.strip()]
        self.rate = float(rospy.get_param('~rate', 10.0))
        self.start = rospy.Time.now().to_sec()
        rospy.wait_for_service('/gazebo/set_model_state')
        self.set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        rospy.loginfo('[sim_people_mover] moving %s at %.1f Hz',
                      ','.join(self.names), self.rate)

    @staticmethod
    def path(i, t):
        if i % 3 == 0:
            x = 3.5 + 1.5 * math.sin(0.50 * t)
            y = 1.2
            vx = 0.75 * math.cos(0.50 * t)
            vy = 0.0
        elif i % 3 == 1:
            x = 4.0
            y = -1.4 + 0.7 * math.sin(0.70 * t)
            vx = 0.0
            vy = 0.49 * math.cos(0.70 * t)
        else:
            x = 5.0 - 1.0 * math.sin(0.35 * t)
            y = 0.5 + 0.6 * math.sin(0.60 * t)
            vx = -0.35 * math.cos(0.35 * t)
            vy = 0.36 * math.cos(0.60 * t)
        yaw = math.atan2(vy, vx) if abs(vx) + abs(vy) > 1e-3 else 0.0
        return x, y, yaw

    def run(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            t = rospy.Time.now().to_sec() - self.start - 3.0
            if t < 0.0:
                rate.sleep()
                continue
            for i, name in enumerate(self.names):
                x, y, yaw = self.path(i, t)
                msg = ModelState()
                msg.model_name = name
                msg.pose.position.x = x
                msg.pose.position.y = y
                msg.pose.position.z = 0.0
                msg.pose.orientation.z = math.sin(yaw * 0.5)
                msg.pose.orientation.w = math.cos(yaw * 0.5)
                try:
                    self.set_state(msg)
                except rospy.ServiceException:
                    pass
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('sim_people_mover')
    PeopleMover().run()
