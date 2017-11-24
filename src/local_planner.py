#!/usr/bin/env python
# -*- coding: utf-8 -*-
import tf
import math
import rospy
from nav_msgs.msg import Path
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

robot_goal = None
cmd_vel_pub = None
land_pub = None

acc_limit_x = 2.5
acc_limit_y = 2.5
max_rot_vel = 1.0
min_rot_vel = 0.2
acc_limit_th = 3.2
max_trans_vel = 0.5
min_trans_vel = 0.1

turn_rate = 45
max_velocity = 5

landed = False

def distance(x1, y1, x2, y2):
    return math.sqrt( (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2))

def lookAt(robotx, roboty, helix, heliy):
    dx = helix - robotx
    dy = heliy - roboty
    return math.atan2(dy,dx)

def init():
    global cmd_vel_pub, land_pub
    global acc_limit_x, acc_limit_y, acc_limit_th
    global max_trans_vel, min_trans_vel, max_rot_vel, min_rot_vel
    global max_velocity, turn_rate
    rospy.init_node('aerial_local_planner')
    acc_limit_x = rospy.get_param("~acc_limit_x", 2.5)
    acc_limit_y = rospy.get_param("~acc_limit_y", 2.5)
    acc_limit_th = rospy.get_param("~acc_limit_th", 3.2)
    max_trans_vel = rospy.get_param("~max_trans_vel", 1.0)
    min_trans_vel = rospy.get_param("~min_trans_vel", 0.1)
    max_rot_vel = rospy.get_param("~max_rot_vel", 1.0)
    min_rot_vel = rospy.get_param("~min_rot_vel", 0.2)
    turn_rate = rospy.get_param("~turn_rate", 45) # Degrees/sec
    max_velocity = rospy.get_param("~max_velocity", 5) # m/s
    #theta_tolerance = rospy.get_param() TODO
    #pos_tolerance = rospy.get_param() TODO
    cmd_vel_pub = rospy.Publisher("bebop/cmd_vel", Twist, queue_size=1)
    land_pub = rospy.Publisher("bebop/reset", Empty, queue_size=1)
    rospy.Subscriber("bebop/odom", Odometry, odom_callback)
    rospy.Subscriber("aerial_global_planner/plan", Path, path_callback)
    while not rospy.is_shutdown():
        rospy.spin()

def odom_callback(odom):
    global cmd_vel_pub, robot_goal, land_pub
    global max_trans_vel, min_trans_vel, max_rot_vel, min_rot_vel
    global landed
    # TODO Here is the main local planner procedure!
    twist = Twist()
    if not landed:
        d = 0.0
        curr_pos = odom.pose.pose.position
        curr_or = odom.pose.pose.orientation
        if robot_goal != None:
            d = distance(curr_pos.x, curr_pos.y, robot_goal.position.x, robot_goal.position.y)
            zd = curr_pos.z - robot_goal.position.z
            #print d
            if d < 0.1 and zd > 0 and zd < 0.2:
                empty = Empty()
                land_pub.publish(empty)
                landed = True
                return
            (gr, gp, gy) = tf.transformations.euler_from_quaternion([robot_goal.orientation.x, robot_goal.orientation.y, robot_goal.orientation.z, robot_goal.orientation.w])
            (rr, rp, ry) = tf.transformations.euler_from_quaternion([curr_or.x, curr_or.y, curr_or.z, curr_or.w])
            yaw_diff = gy - ry

            x_diff = robot_goal.position.x - curr_pos.x # forward is positive
            y_diff = robot_goal.position.y - curr_pos.y # left is positive
            z_diff = robot_goal.position.z - (curr_pos.z - 0.1) # up is positive 
            x_diff /= 1
            y_diff /= 1
            z_diff /= 1
            yaw_diff /= 1

            straight_yaw = lookAt(curr_pos.x, curr_pos.y, robot_goal.position.x, robot_goal.position.y)
            yaw_diff2 = straight_yaw - ry
            # polar coordinates r,θ
            # θ = yaw_diff2
            # r = d
            x_diff = d * math.cos(yaw_diff2)
            y_diff = d * math.sin(yaw_diff2)


            '''
            if x_diff > min_trans_vel:
                x_diff = max_trans_vel
            elif x_diff < 0 and x_diff < -min_trans_vel:
                x_diff = -max_trans_vel
            else:
                x_diff = 0
            if y_diff > min_trans_vel:
                y_diff = max_trans_vel
            elif y_diff < 0 and y_diff < -min_trans_vel:
                y_diff = -max_trans_vel
            else:
                y_diff = 0
            '''


            # TODO take HEADING into account
            if x_diff > 0:
                x_diff = max_trans_vel
            elif x_diff < -0:
                x_diff = -max_trans_vel
            if y_diff > 0:
                y_diff = max_trans_vel
            if y_diff < -0:
                y_diff = -max_trans_vel
            if z_diff > max_trans_vel:
                z_diff = max_trans_vel
            if z_diff < -max_trans_vel:
                z_diff = -max_trans_vel
            twist.linear.x = x_diff
            twist.linear.y = y_diff
            twist.linear.z = z_diff
            twist.angular.z = yaw_diff2*10
            #print '-----------'
            #print straight_yaw
            #print ry
            #print yaw_diff2
            #print '-----------'
            # TODO act based on distance (and time?)
    cmd_vel_pub.publish(twist)

# Maybe I can just head towards the tf instead of subscribing to the path
def path_callback(path):
    global robot_goal
    # TODO delete the code below and add a tf listener that
    # will transform the path's coordinates from helipad
    # to base_link!
    robot_goal = path.poses[0].pose
    #print robot_goal

if __name__ == '__main__':
    init()