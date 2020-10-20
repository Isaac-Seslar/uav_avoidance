#! /usr/bin/python

import math
import rospy
import mavros
import std_msgs
import tf
import sys, select, os, tty, termios

from geometry_msgs.msg import PoseStamped, TwistStamped, TransformStamped
from numpy import linspace, array
from mavros_msgs.msg import GlobalPositionTarget, State, HomePosition
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL

class vicon_quad:
        quad_pose   = PoseStamped()
#       quad_pose  = PoseStamped()
        quad_pose_x = PoseStamped()
        quad_pose_y = PoseStamped()
        quad_pose_z = PoseStamped()
	publish_val = 1

def vicon_callback(msg):
        quad_obj.quad_pose = msg
        p.header             =  quad_obj.quad_pose.header
	p.header.stamp = rospy.Time.now()
	p.header.frame_id = "earth"
	p.geo.latitude = 3
	p.geo.longitude = -106
	p.geo.altitude = 1
        p.orientation   =  quad_obj.quad_pose.pose.orientation
        p.position      =  quad_obj.quad_pose.pose.position
	p.position.z = 0
	p.approach.x = 0
	p.approach.y = 0
	p.approach.z = 0     
	pose_pub.publish(p)
	
if __name__ == '__main__':

        rospy.init_node('set_home_sub', anonymous=False)
        quad_obj = vicon_quad()
	r = rospy.Rate(50) # 50hz
        # Subscribers
        vicon_sub = rospy.Subscriber('/mavros/local_position/pose',
                PoseStamped, vicon_callback, queue_size=10)
        #vicon_sub = rospy.Subscriber('/vicon/QuadrotorMark/QuadrotorMark',
        #       TransformStamped, vicon_callback, queue_size=10)

        # Publishers
        pose_pub = rospy.Publisher('/mavros/home_position/home', HomePosition,
                 queue_size=1)

        p = HomePosition()
	setpnt_pub  = rospy.Publisher('/mavros/setpoint_position/local',PoseStamped, queue_size=10)

	quad_pose=PoseStamped()

	quad_pose.pose.position.x = 0
	quad_pose.pose.position.y = 0
	quad_pose.pose.position.z = 0

	for i in range(50):
		setpnt_pub.publish(quad_pose)
		r.sleep()
        while not rospy.is_shutdown():
		pass	
         #r.sleep()

