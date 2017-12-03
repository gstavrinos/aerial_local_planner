#!/usr/bin/env python
# -*- coding: utf-8 -*-
import tf
import math
import rospy
from nav_msgs.msg import Path
from std_msgs.msg import Empty
from tf import TransformListener
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist

robot_goal = None
cmd_vel_pub = None
land_pub = None

z_tolerance = 0.0
xy_tolerance = 0.0
max_trans_vel = 0.5
min_trans_vel = 0.1

landed = False

tf_ = None
goal_prediction_pose = None

def distance(x1, y1, x2, y2):
    return math.sqrt( (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2))

def lookAt(robotx, roboty, helix, heliy):
    dx = helix - robotx
    dy = heliy - roboty
    return math.atan2(dy,dx)

def init():
    global cmd_vel_pub, land_pub
    global max_trans_vel, min_trans_vel
    global  tf_, xy_tolerance, z_tolerance
    rospy.init_node('aerial_local_planner')
    max_trans_vel = rospy.get_param('~max_trans_vel', 5.0)
    min_trans_vel = rospy.get_param('~min_trans_vel', 0.1)
    xy_tolerance = rospy.get_param('~xy_tolerance', 0.05)
    z_tolerance = rospy.get_param('~z_tolerance', 0.2)
    #theta_tolerance = rospy.get_param() TODO
    #pos_tolerance = rospy.get_param() TODO
    cmd_vel_pub = rospy.Publisher('bebop/cmd_vel', Twist, queue_size=1)
    land_pub = rospy.Publisher('bebop/reset', Empty, queue_size=1)
    rospy.Subscriber('bebop/odom', Odometry, odom_callback)
    rospy.Subscriber('aerial_global_planner/plan', Path, path_callback)
    tf_ = TransformListener()
    rospy.Subscriber('tf', TFMessage, tf_callback)
    while not rospy.is_shutdown():
        rospy.spin()

def odom_callback(odom):
    global cmd_vel_pub, robot_goal, land_pub
    global max_trans_vel, min_trans_vel
    global landed, goal_prediction_pose, xy_tolerance, z_tolerance
    twist = Twist()
    if not landed and robot_goal != None:
        d = 0.0
        curr_pos = odom.pose.pose.position
        curr_or = odom.pose.pose.orientation
        if robot_goal != None:
            d = distance(curr_pos.x, curr_pos.y, robot_goal.position.x, robot_goal.position.y)
            d2 = 1000
            zd2 = 1000
            if goal_prediction_pose != None:
                d2 = distance(curr_pos.x, curr_pos.y, goal_prediction_pose.position.x, goal_prediction_pose.position.y)
                zd2 = curr_pos.z - goal_prediction_pose.position.z
            zd = abs(curr_pos.z - robot_goal.position.z)
            if (d2 < 0.2 and zd2 < z_tolerance and zd2 > 0.0):
                empty = Empty()
                land_pub.publish(empty)
                landed = True
                return
            if not (d <= xy_tolerance and zd < z_tolerance and zd2):
                (gr, gp, gy) = tf.transformations.euler_from_quaternion([robot_goal.orientation.x, robot_goal.orientation.y, robot_goal.orientation.z, robot_goal.orientation.w])
                (rr, rp, ry) = tf.transformations.euler_from_quaternion([curr_or.x, curr_or.y, curr_or.z, curr_or.w])
                yaw_diff = gy - ry

                yaw_diff /= 1

                straight_yaw = lookAt(curr_pos.x, curr_pos.y, robot_goal.position.x, robot_goal.position.y)
                yaw_diff2 = straight_yaw - ry

                # polar coordinates r,θ
                # θ = yaw_diff2
                # r = d
                x_diff = 2 * d * math.cos(yaw_diff2)
                y_diff = 2 * d * math.sin(yaw_diff2)
                z_diff = 2 * (robot_goal.position.z - (curr_pos.z - 0.1)) # up is positive

                x_diff = max(min_trans_vel, min(x_diff,max_trans_vel))
                y_diff = max(min_trans_vel, min(y_diff,max_trans_vel))

                twist.linear.x = x_diff
                twist.linear.y = y_diff
                twist.linear.z = z_diff
                twist.angular.z = yaw_diff2*10
    cmd_vel_pub.publish(twist)

def path_callback(path):
    global robot_goal
    if len(path.poses) > 0:
        robot_goal = path.poses[0].pose
    else:
        robot_goal = None

def tf_callback(tf2):
    global goal_prediction_pose, tf_
    try:
        t = tf_.getLatestCommonTime('/odom', '/goal_prediction')
        position, quaternion = tf_.lookupTransform('/odom', '/goal_prediction', t)
        goal_prediction_pose = Pose()
        goal_prediction_pose.position.x = position[0]
        goal_prediction_pose.position.y = position[1]
        goal_prediction_pose.position.z = position[2]
    except Exception as e:
        pass

if __name__ == '__main__':
    init()