#!/usr/bin/env python
from __future__ import print_function
import sys, threading, time
import actionlib, rospy, tf2_ros
from geometry_msgs.msg import Pose2D, TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Float64, String
from xzrobot_ros_dependency.msg import NaviCmdActionAction, NaviCmdActionGoal

ACTION = '/navtest/RobotNaviCmdAction'
last = {'map': None, 'area': None, 'status': None, 'feedback': None}
checks = []

def check(name, ok, detail=''):
    print(('[PASS] ' if ok else '[FAIL] ') + name + (' ' + detail if detail else ''))
    checks.append(ok)

def on_map(msg): last['map'] = msg
def on_area(msg): last['area'] = msg.data
def on_status(msg): last['status'] = msg.data
def on_feedback(msg): last['feedback'] = msg

rospy.init_node('coverage_smoke')
br = tf2_ros.StaticTransformBroadcaster()
t = TransformStamped(); t.header.stamp = rospy.Time.now(); t.header.frame_id='map'; t.child_frame_id='base_footprint'; t.transform.rotation.w=1.0
br.sendTransform(t)

pub = rospy.Publisher('/map', OccupancyGrid, queue_size=1, latch=True)
grid = OccupancyGrid(); grid.header.frame_id='map'; grid.info.resolution=0.05
grid.info.width=200; grid.info.height=200
grid.info.origin.position.x=-5.0; grid.info.origin.position.y=-5.0; grid.info.origin.orientation.w=1.0
grid.data=[0]*(200*200)
while pub.get_num_connections()==0 and not rospy.is_shutdown(): rospy.sleep(0.1)
pub.publish(grid)

odom_pub = rospy.Publisher('/odom', Odometry, queue_size=1)
def odom_loop():
    r=rospy.Rate(20)
    while not rospy.is_shutdown():
        o=Odometry(); o.header.stamp=rospy.Time.now(); o.header.frame_id='map'; o.child_frame_id='base_footprint'; o.pose.pose.orientation.w=1.0
        try: odom_pub.publish(o)
        except rospy.ROSException: break
        r.sleep()
th=threading.Thread(target=odom_loop); th.setDaemon(True); th.start()

rospy.Subscriber('/navtest/clean_map', OccupancyGrid, on_map)
rospy.Subscriber('/navtest/clean_area', Float64, on_area)
rospy.Subscriber('/navtest/clean_part_status', String, on_status)

client=actionlib.SimpleActionClient(ACTION, NaviCmdActionAction)
if not client.wait_for_server(rospy.Duration(20)):
    print('[FAIL] action server missing'); sys.exit(2)
goal=NaviCmdActionGoal(); goal.navi_cmd=20; goal.robot_vel=0.3
for (x,y) in [(-0.15,-0.15),(0.15,-0.15),(0.15,0.15),(-0.15,0.15)]:
    p=Pose2D(); p.x=x; p.y=y; p.theta=0.0; goal.tracking_path.append(p)
client.send_goal(goal, feedback_cb=on_feedback)
if not client.wait_for_result(rospy.Duration(15)):
    print('[FAIL] no result'); sys.exit(1)
res=client.get_result()
print('result=%d percent=%.3f clean_area=%.4f state=%s' % (
    res.tracking_path_result, res.execution_percent, res.clean_area, client.get_state()))
check('result=1', res.tracking_path_result == 1, str(res.tracking_path_result))
check('coverage_rate>=0.99', res.execution_percent >= 0.99, '%.3f' % res.execution_percent)
check('clean_area>0.05', res.clean_area > 0.05, '%.4f' % res.clean_area)

rospy.sleep(0.5)
check('clean_map received', last['map'] is not None)
if last['map'] is not None:
    covered = sum(1 for v in last['map'].data if v == 50)
    check('clean_map has covered cells', covered > 0, 'covered=%d' % covered)
check('clean_area topic', last['area'] is not None and last['area'] > 0.05, str(last['area']))
check('status DONE', last['status'] is not None and '"state":"DONE"' in last['status'], str(last['status'])[:120])
print('ALL PASS' if all(checks) else 'FAILED')
sys.exit(0 if all(checks) else 1)
