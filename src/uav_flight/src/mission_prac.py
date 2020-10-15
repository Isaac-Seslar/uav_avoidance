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

class quad_obj:
	quad_pose   = TransformStamped()
	quad_pose_x = TransformStamped()
	quad_pose_y = TransformStamped()
	quad_pose_z = TransformStamped()

	state         = State()
	quad_goal     = PoseStamped()
	offb_set_mode = SetMode()
	arm_cmd       = CommandBool()
	takeoff_cmd   = CommandTOL()
	landing_cmd   = CommandTOL()


def pose_callback(ps_msg):
	quad_obj.quad_pose = ps_msg

def state_callback(st_msg):
	quad_obj.state = st_msg

def isclose(a, b, rel_tol=1e-09, abs_tol=0.0):
    return abs(a-b) <= max(rel_tol * max(abs(a), abs(b)), abs_tol)


if __name__ == '__main__':

    rospy.init_node('mission', anonymous=False)

    # Publishers
    setpnt_pub  = rospy.Publisher('/mavros/setpoint_position/local',
        			  				PoseStamped, queue_size=10)
    # velcty_pub  = rospy.ServiceProxy('/mavroslocal')

    # Subscribers
    state_sub   = rospy.Subscriber('/mavros/state', State, 
        							state_callback, queue_size=10)

    pose_sub    = rospy.Subscriber('/mavros/vision_pose/pose', PoseStamped, 
									pose_callback, queue_size=10) 


    # Services
    setmod_serv  = rospy.ServiceProxy('/mavros/set_mode', SetMode)
    arming_serv  = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
    takeoff_serv = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
    landing_serv = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)

    # Time setup
    now       = rospy.Time.now()
    zero_time = rospy.Time()
    rate      = rospy.Rate(50) # This node will publish at 50Hz       	

    while not rospy.is_shutdown():
   		setpnt_pub.publish(quad_obj.quad_goal)
   		rate.sleep()
   		if quad_obj.state.mode=="OFFBOARD":
   			if not quad_obj.state.armed==True:
   				print "Arming..."
   				rospy.wait_for_service('/mavros/cmd/arming')
   				arming_serv(True)
   				rospy.sleep(2)

   			print "Taking Off..."
   			rospy.wait_for_service('/mavros/cmd/takeoff')
			takeoff_serv(altitude=1.5)
   			rospy.sleep(5)
   			break

   			# if takeoff_serv.success==True:
   			# 	break

	# Get current position
    # cur_posx = quad_obj.quad_pose.pose.position.x
    # cur_posy = quad_obj.quad_pose.pose.position.y
    # cur_posz = quad_obj.quad_pose.pose.position.z

   	# Start mission #
   	#################
    origin = [0, 0, 1] # Somewhat close to the middle of the cage

   	# Wherever the quad is send it back to the middle
   	# Check tolerances to origin, see if close enough to origin to start mission
    # x_tol = isclose(cur_posx, origin[0], abs_tol=0.25)
    # y_tol = isclose(cur_posy, origin[1], abs_tol=0.25)
    # z_tol = isclose(cur_posz, origin[2], abs_tol=0.25)
    # toles = all([x_tol, y_tol, z_tol])
    rospy.wait_for_service('/mavros/set_mode')
    while not quad_obj.state.mode=="OFFBOARD":
    	setpnt_pub.publish(quad_obj.quad_goal)
    	setmod_serv(custom_mode="OFFBOARD")
    	rate.sleep()

    waypnt_x = [-1,-1,-1,0,1,1,1,0]
    waypnt_y = [ 1,0,-1,-1,-1,0,1,1]
    print "Starting Mission"
		# Circle flight

    # while not rospy.is_shutdown():

		# Update current position
    for i in range(len(waypnt_x)):

		quad_obj.quad_goal.pose.position.x = waypnt_x[i]
		quad_obj.quad_goal.pose.position.y = waypnt_y[i]
		quad_obj.quad_goal.pose.position.z = 1.5

		for ii in range(100):
			setpnt_pub.publish(quad_obj.quad_goal)
			rate.sleep()

			# cur_posx = quad_obj.quad_pose.pose.position.x
		# cur_posy = quad_obj.quad_pose.pose.position.y
		# cur_posz = quad_obj.quad_pose.pose.position.z

    print "Landing"
    landing_serv(altitude=0)
    rospy.sleep(10)
		

			

    print "done"


    ##### This is meant to bring the quad back to the center to start the mission ####
    #### Doesn't quite work

      #   while toles==False:

		# # Update current position
		# cur_posx = quad_obj.quad_pose.pose.position.x
		# cur_posy = quad_obj.quad_pose.pose.position.y
		# cur_posz = quad_obj.quad_pose.pose.position.z

		# print(cur_posx,cur_posy,cur_posz)

		# x_tol = isclose(cur_posx, origin[0], abs_tol=0.5)
		# y_tol = isclose(cur_posy, origin[1], abs_tol=0.5)
		# z_tol = isclose(cur_posz, origin[2], abs_tol=0.5)
		# toles = all([x_tol, y_tol, z_tol])

		# # Adjust current position to go closer to the origin
		# if cur_posx > origin[0]:
		# 	quad_obj.quad_goal.pose.position.x = cur_posx + 0.25
		# if cur_posy > origin[1]:
		# 	quad_obj.quad_goal.pose.position.y = cur_posy + 0.25
		# if cur_posz > origin[2]:
		# 	quad_obj.quad_goal.pose.position.z = cur_posz + 0.2
		# if cur_posx < origin[0]:
		# 	quad_obj.quad_goal.pose.position.x = cur_posx - 0.25
		# if cur_posy < origin[1]:
		# 	quad_obj.quad_goal.pose.position.y = cur_posy - 0.25
		# if cur_posz < origin[2]:
		# 	quad_obj.quad_goal.pose.position.z = cur_posz - 0.2

		# print(quad_obj.quad_goal)
		# setpnt_pub.publish(quad_obj.quad_goal)
		# rate.sleep()
		# print("Adjusting...")

      	#### This will work once takeoff using MoCap is figured out ###
      	###############################################################

        # for i in range(10):
        # 	print quad_obj.state
        # 	rate.sleep()


       	# while not rospy.is_shutdown() and quad_obj.state.armed==False:
       	# 	setpnt_pub.publish(quad_obj.quad_goal)
       	# 	rate.sleep()
       	# 	if quad_obj.state.mode =="OFFBOARD":
       	# 		arming_serv(True)
       	# 		rate.sleep()
       	# 	if quad_obj.state.armed==True:
       	# 		print "Armed"
       	# 		rate.sleep()
       	# 		now = rospy.Time.now()

       	# rospy.sleep(2)
       	# print("Takeoff initiated...")
       	# # rate.sleep()

       	# rospy.wait_for_service('/mavros/cmd/takeoff')
       	# rospy.wait_for_service('/mavros/cmd/land')
       	# takeoff_serv(min_pitch=0,yaw=0,latitude=47.3977,longitude=8.54559,altitude=1)

       	# rospy.sleep(5)
    
       	# quad_obj.quad_goal.pose.position.x = 0
       	# quad_obj.quad_goal.pose.position.y = 0
       	# quad_obj.quad_goal.pose.position.z = 1

       	# for i in range(10):
       	# 	setpnt_pub.publish(quad_obj.quad_goal)
       	# 	print "???"

       	# rospy.sleep(7)
       	# # landing_serv()
       	# # rospy.sleep(5)