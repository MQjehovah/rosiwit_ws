#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Shutdown guard: when roslaunch receives Ctrl+C, make sure Gazebo dies.

The openni depth camera plugin on WSL can hang during Gazebo shutdown, leaving
gzserver/gzclient behind after roslaunch exits. rospy.on_shutdown runs before
this node exits, so a hard pkill here guarantees a clean Ctrl+C.
"""
import subprocess

import rospy


def kill_gazebo():
    for name in ('gzserver', 'gzclient'):
        try:
            subprocess.call(['pkill', '-9', '-x', name])
        except OSError:
            pass
    rospy.loginfo('[sim_shutdown_guard] gzserver/gzclient killed')


if __name__ == '__main__':
    rospy.init_node('sim_shutdown_guard')
    rospy.on_shutdown(kill_gazebo)
    rospy.spin()
