#!/usr/bin/env python
import tf
import math
import rospy
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

robot_goal = None
cmd_vel_pub = None

acc_limit_x = 2.5
acc_limit_y = 2.5
max_rot_vel = 1.0
min_rot_vel = 0.2
acc_limit_th = 3.2
max_trans_vel = 0.5
min_trans_vel = 0.1

turn_rate = 45
max_velocity = 5

def distance(x1, y1, x2, y2):
    return math.sqrt( (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2))

def init():
    global cmd_vel_pub
    global acc_limit_x, acc_limit_y, acc_limit_th
    global max_trans_vel, min_trans_vel, max_rot_vel, min_rot_vel
    global max_velocity, turn_rate
    rospy.init_node('aerial_local_planner')
    acc_limit_x = rospy.get_param("~acc_limit_x", 2.5)
    acc_limit_y = rospy.get_param("~acc_limit_y", 2.5)
    acc_limit_th = rospy.get_param("~acc_limit_th", 3.2)
    max_trans_vel = rospy.get_param("~max_trans_vel", 0.5)
    min_trans_vel = rospy.get_param("~min_trans_vel", 0.1)
    max_rot_vel = rospy.get_param("~max_rot_vel", 1.0)
    min_rot_vel = rospy.get_param("~min_rot_vel", 0.2)
    turn_rate = rospy.get_param("~turn_rate", 45) # Degrees/sec
    max_velocity = rospy.get_param("~max_velocity", 5) # m/s
    #theta_tolerance = rospy.get_param() TODO
    #pos_tolerance = rospy.get_param() TODO
    cmd_vel_pub = rospy.Publisher("bebop/cmd_vel", Twist, queue_size=1)
    rospy.Subscriber("bebop/odom", Odometry, odom_callback)
    rospy.Subscriber("aerial_global_planner/plan", Path, path_callback)
    while not rospy.is_shutdown():
        rospy.spin()

def odom_callback(odom):
    global cmd_vel_pub, robot_goal
    # TODO Here is the main local planner procedure!
    #print msg
    d = 0.0
    twist = Twist()
    curr_pos = odom.pose.pose.position
    curr_or = odom.pose.pose.orientation
    if robot_goal != None:
        d = distance(curr_pos.x, curr_pos.y, robot_goal.position.x, robot_goal.position.y)
        (gr, gp, gy) = tf.transformations.euler_from_quaternion([robot_goal.orientation.x, robot_goal.orientation.y, robot_goal.orientation.z, robot_goal.orientation.w])
        (rr, rp, ry) = tf.transformations.euler_from_quaternion([curr_or.x, curr_or.y, curr_or.z, curr_or.w])
        yaw_diff = gy - ry # TODO Check that counter clockwise is positive
        x_diff = robot_goal.position.x - curr_pos.x # forward is positive
        y_diff = robot_goal.position.y - curr_pos.y # left is positive
        z_diff = robot_goal.position.z - curr_pos.z # up is positive 
        # TODO act based on distance (and time?)
    cmd_vel_pub.publish(twist)

# Maybe I can just head towards the tf instead of subscribing to the path
def path_callback(path):
    global robot_goal
    # TODO delete the code below and add a tf listener that
    # will transform the path's coordinates from helipad
    # to base_link!
    robot_goal = path.poses[0].pose
    print robot_goal

if __name__ == '__main__':
    init()