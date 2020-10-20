#! /usr/bin/python

import math
import rospy
import mavros
import std_msgs
import tf
import sys, select, os, tty, termios

from geometry_msgs.msg import PoseStamped, TwistStamped, TransformStamped
from numpy import linspace, array
from mavros_msgs.msg import GlobalPositionTarget, State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL

class vicon_quad:
        quad_pose   = TransformStamped()
#       quad_pose  = PoseStamped()
        quad_pose_x = TransformStamped()
        quad_pose_y = TransformStamped()
        quad_pose_z = TransformStamped()
	publish_val = 1

def vicon_callback(msg):
        quad_obj.quad_pose = msg
        p.header             =  quad_obj.quad_pose.header
        p.pose.orientation   =  quad_obj.quad_pose.transform.rotation
        p.pose.position      =  quad_obj.quad_pose.transform.translation
	if quad_obj.publish_val == 2:        
		pose_pub.publish(p)
		quad_obj.publish_val = 1
	else:
		quad_obj.publish_val = quad_obj.publish_val + 1
	
if __name__ == '__main__':

        rospy.init_node('vicon_sub', anonymous=False)
        quad_obj = vicon_quad()
	#r = rospy.Rate(50) # 50hz
        # Subscribers
        vicon_sub = rospy.Subscriber('/vicon/Odroid/Odroid',
                TransformStamped, vicon_callback, queue_size=10)
        #vicon_sub = rospy.Subscriber('/vicon/QuadrotorMark/QuadrotorMark',
        #       TransformStamped, vicon_callback, queue_size=10)

        # Publishers
        pose_pub = rospy.Publisher('/mavros1/vision_pose/pose',PoseStamped,
                 queue_size=10)

        p = PoseStamped()
	
        while not rospy.is_shutdown():
		pass
                #r.sleep()

