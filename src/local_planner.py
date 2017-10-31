#!/usr/bin/env python
import rospy
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

robot_goal = None
cmd_vel_pub = None

def init():
    global cmd_vel_pub
    rospy.init_node('aerial_local_planner')
    cmd_vel_pub = rospy.Publisher("bebop/cmd_vel", Twist, queue_size=1)
    rospy.Subscriber("bebop/odom", Odometry, odom_callback)
    rospy.Subscriber("aerial_global_planner/plan", Path, path_callback)
    while not rospy.is_shutdown():
        rospy.spin()

def odom_callback(msg):
    global cmd_vel_pub
    # TODO Here is the main local planner procedure!
    #print msg
    twist = Twist()
    cmd_vel_pub.publish(twist)


def path_callback(path):
    global robot_goal
    # TODO delete the code below and add a tf listener that
    # will transform the path's coordinates from helipad
    # to base_link!
    robot_goal = path.poses[0]
    print robot_goal

if __name__ == '__main__':
    init()