#!/usr/bin/env python
from __future__ import print_function
import math, sys, threading, time
import actionlib, rospy, tf2_ros
from geometry_msgs.msg import Pose2D, TransformStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Float64, String
from xzrobot_ros_dependency.msg import NaviCmdActionAction, NaviCmdActionGoal

ACTION='/navtest/RobotNaviCmdAction'
pose = {'x':0.0,'y':0.0,'theta':0.0}
last_cmd = {'v':0.0,'w':0.0}
last = {'map':None,'area':None,'status':None,'feedback':None}
lock = threading.Lock()

def on_cmd(msg):
    with lock: last_cmd['v']=msg.linear.x; last_cmd['w']=msg.angular.z
def on_map(msg): last['map']=msg
def on_area(msg): last['area']=msg.data
def on_status(msg): last['status']=msg.data
def on_fb(msg): last['feedback']=msg

rospy.init_node('coverage_drive_smoke')
br = tf2_ros.TransformBroadcaster()
odom_pub = rospy.Publisher('/odom', Odometry, queue_size=1)
rospy.Subscriber('/navtest/movebase_cmd_vel', Twist, on_cmd)
rospy.Subscriber('/navtest/clean_map', OccupancyGrid, on_map)
rospy.Subscriber('/navtest/clean_area', Float64, on_area)
rospy.Subscriber('/navtest/clean_part_status', String, on_status)

def drive_loop():
    rate=rospy.Rate(50); last_t=time.time()
    while not rospy.is_shutdown():
        now=time.time(); dt=min(0.1, now-last_t); last_t=now
        with lock:
            v=last_cmd['v']; w=last_cmd['w']
        pose['x'] += v*math.cos(pose['theta'])*dt
        pose['y'] += v*math.sin(pose['theta'])*dt
        pose['theta'] += w*dt
        qz=math.sin(pose['theta']*0.5); qw=math.cos(pose['theta']*0.5)
        tf=TransformStamped(); tf.header.stamp=rospy.Time.now(); tf.header.frame_id='map'; tf.child_frame_id='base_footprint'
        tf.transform.translation.x=pose['x']; tf.transform.translation.y=pose['y']
        tf.transform.rotation.z=qz; tf.transform.rotation.w=qw
        try:
            br.sendTransform(tf)
        except rospy.ROSException:
            break
        o=Odometry(); o.header.stamp=rospy.Time.now(); o.header.frame_id='map'; o.child_frame_id='base_footprint'
        o.pose.pose.position.x=pose['x']; o.pose.pose.position.y=pose['y']
        o.pose.pose.orientation.z=qz; o.pose.pose.orientation.w=qw
        o.twist.twist.linear.x=v; o.twist.twist.angular.z=w
        odom_pub.publish(o)
        rate.sleep()

pub=rospy.Publisher('/map',OccupancyGrid,queue_size=1,latch=True)
grid=OccupancyGrid(); grid.header.frame_id='map'; grid.info.resolution=0.05
grid.info.width=200; grid.info.height=200
grid.info.origin.position.x=-5.0; grid.info.origin.position.y=-5.0; grid.info.origin.orientation.w=1.0
grid.data=[0]*(200*200)
while pub.get_num_connections()==0 and not rospy.is_shutdown(): rospy.sleep(0.1)
pub.publish(grid)
th=threading.Thread(target=drive_loop); th.setDaemon(True); th.start()

client=actionlib.SimpleActionClient(ACTION,NaviCmdActionAction)
if not client.wait_for_server(rospy.Duration(20)):
    print('[FAIL] action server missing'); sys.exit(2)
goal=NaviCmdActionGoal(); goal.navi_cmd=20; goal.robot_vel=0.35
for (x,y) in [(-1.0,-0.5),(1.0,-0.5),(1.0,0.5),(-1.0,0.5)]:
    p=Pose2D(); p.x=x; p.y=y; goal.tracking_path.append(p)
client.send_goal(goal, feedback_cb=on_fb)
start=time.time()
res=None
while time.time()-start < 100.0 and not rospy.is_shutdown():
    if client.wait_for_result(rospy.Duration(1.0)):
        res=client.get_result(); break
    fb=last['feedback']
    if fb is not None and int(time.time()-start) % 10 == 0:
        print('t=%ds pose=(%.2f,%.2f) rate=%.3f area=%.2f state=%s' % (
            int(time.time()-start), pose['x'], pose['y'], fb.execution_percent,
            fb.clean_area, client.get_state()))
if res is None:
    print('[FAIL] timeout, last pose=(%.2f,%.2f) rate=%s' % (
        pose['x'], pose['y'], None if last['feedback'] is None else last['feedback'].execution_percent))
    sys.exit(1)
print('result=%d percent=%.3f clean_area=%.3f state=%s' % (
    res.tracking_path_result, res.execution_percent, res.clean_area, client.get_state()))
print('final pose=(%.2f,%.2f,%.2f)' % (pose['x'],pose['y'],pose['theta']))
print('status=%s' % (str(last['status'])[:300]))
ok = (res.tracking_path_result == 1 and res.execution_percent >= 0.95 and
      last['status'] is not None and '"state":"DONE"' in last['status'])
print('ALL PASS' if ok else 'FAILED')
sys.exit(0 if ok else 1)
