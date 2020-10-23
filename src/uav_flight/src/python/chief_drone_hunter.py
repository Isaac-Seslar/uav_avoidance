#! /usr/bin/python

####################
# Import libraries #
####################

import math
import numpy as np
import rospy
import mavros
import std_msgs
import tf 
import sys, select, os, tty, termios

from geometry_msgs.msg import PoseStamped, TwistStamped, TransformStamped, Point, Vector3
from mavros_msgs.msg import GlobalPositionTarget, State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL

# Personal def
# from Common_Functions import *
# from Safe_Controller import *
# from Performance_Controller import *

#############
# Class Def #
#############

class obs_obj:
	obs_l = Vector3()
	obs_s = Vector3()

class quad_obj:
	vehicle = Point()
	float_x = float() 
	float_y = float()
	float_z = float()

	vehicle_dot = Vector3()
	float_xdot  = float()
	float_ydot  = float()
	float_zdot  = float()

	state         = State()
	quad_goal     = PoseStamped()
	offb_set_mode = SetMode()
	arm_cmd       = CommandBool()
	takeoff_cmd   = CommandTOL()
	landing_cmd   = CommandTOL()

###############
# Definitions #
###############

# Actively watches for keypress #
#################################
def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
 
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

# Tolerance function #
######################
def isclose(a, b, rel_tol=1e-09, abs_tol=0.0):
    return abs(a-b) <= max(rel_tol * max(abs(a), abs(b)), abs_tol)

# Callback functions for local pose #
#####################################
def pose_callback(msg):
    quad_obj.vehicle.x = msg.pose.position.x
    quad_obj.vehicle.y = msg.pose.position.y
    quad_obj.vehicle.z = msg.pose.position.z

    quad_obj.float_x = float(msg.pose.position.x) 
    quad_obj.float_y = float(msg.pose.position.y)
    quad_obj.float_z = float(msg.pose.position.z)

def velocity_callback(msg):
	quad_obj.vehicle_dot.x = msg.twist.linear.x
	quad_obj.vehicle_dot.y = msg.twist.linear.y
	quad_obj.vehicle_dot.z = msg.twist.linear.z

	quad_obj.float_xdot = float(msg.twist.linear.x)
	quad_obj.float_ydot = float(msg.twist.linear.y)
	quad_obj.float_zdot = float(msg.twist.linear.z)

def obs_l_callback(msg):
	obs_obj.obs_l = msg.transform.translation
	
def obs_s_callback(msg):
	obs_obj.obs_s = msg.transform.translation

def state_callback(st_msg):
	quad_obj.state = st_msg
	
# Generates waypoints for the quadrotor to follow #
###################################################
def mission():
	# Defines where the obstacles are on the grid
	obstacle_list = np.array([[obs_obj.obs_l.x,obs_obj.obs_l.y],
							[obs_obj.obs_s.x,obs_obj.obs_s.y]])

	

	# obstacle_list = np.array([1,1],[2,-1],[3,3])

	# Initialize values for RTA_Gate()
	current_mode  = 0
	safe_complete = 0

	# Initialize the first waypoint. This will be updated in the loop
	cur_wpt = np.array([quad_obj.float_x,quad_obj.float_y])
	x_rta   = cur_wpt[0]
	y_rta   = cur_wpt[1]

	# Set up tolerances for ending the loop
	# x_tol = isclose(quad_obj.float_x, x_targ, abs_tol=0.1)
	# y_tol = isclose(quad_obj.float_y, y_targ, abs_tol=0.1)

	char = 1
	# print char

	while not rospy.is_shutdown():
		small_obs = np.array([obs_obj.obs_s.x,obs_obj.obs_s.y])
		wpt = Performance_Controller(quad_obj.vehicle.x, quad_obj.vehicle.y, small_obs)

		quad_obj.quad_goal.pose.position.x = wpt[0]
		quad_obj.quad_goal.pose.position.y = wpt[1]
		quad_obj.quad_goal.pose.position.z = 1

		# x_tol = isclose(quad_obj.float_x, x_targ, abs_tol=0.1)
		# y_tol = isclose(quad_obj.float_y, y_targ, abs_tol=0.1)
		# print x_rta, quad_obj.float_x, x_targ, x_tol 
		# print x_tol 

		setpnt_pub.publish(quad_obj.quad_goal)

		rate.sleep()
		

def Performance_Controller(vehicle_x, vehicle_y, target):
    # Compute a line from vehicle to target
    slope = (target[1] - vehicle_y)/(target[0] - vehicle_x)

    direction = np.sign(target[0] - vehicle_x)

    step = 1.0

    x_next = vehicle_x + direction*step
    if direction > 0 and x_next < target[0]:
        x_dif = step
    elif direction > 0 and x_next >= target[0]:
        x_dif = target[0] - vehicle_x
    elif direction < 0 and x_next > target[0]:
        x_dif = -step
    else:
        x_dif = target[0] - vehicle_x

    y_next = vehicle_y + slope*x_dif
    x_next = vehicle_x + x_dif

    return np.array([x_next, y_next])


#########
# Start #
#########

if __name__ == '__main__':

	rospy.init_node('RTA_Gate_chief', anonymous=False)

	rate = rospy.Rate(50) 

	# Time setup
	now       = rospy.Time.now()
	zero_time = rospy.Time()

	# Subscribers
	state_sub      = rospy.Subscriber('/chief/state', State, state_callback, queue_size=10)
	quad_pose_sub  = rospy.Subscriber('/chief/local_position/pose', PoseStamped,
	                            pose_callback, queue_size=10)
	velo_sub       = rospy.Subscriber('/chief/local_position/velocity', TwistStamped,
	                            velocity_callback, queue_size=10)

	obs_l_pose_sub = rospy.Subscriber('/vicon/obs_l/obs_l', TransformStamped,
	                            obs_l_callback, queue_size=10)
	obs_s_pose_sub = rospy.Subscriber('/vicon/obs_s/obs_s', TransformStamped,
	                            obs_s_callback, queue_size=10)


	# Publishers
	setpnt_pub     = rospy.Publisher('/chief/setpoint_position/local', PoseStamped, queue_size=10)


	# velocity_pub   = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
	# local_position = rospy.Publisher('/mavros/local_position/pose', PoseStamped)

	# Services
	setmod_serv  = rospy.ServiceProxy('/chief/set_mode', SetMode)
	arming_serv  = rospy.ServiceProxy('/chief/cmd/arming', CommandBool)
	takeoff_serv = rospy.ServiceProxy('/chief/cmd/takeoff', CommandTOL)
	landing_serv = rospy.ServiceProxy('/chief/cmd/land', CommandTOL)

	print "Track?"

	choice = raw_input('yes or no: ')

	if choice=='no' or choice=='n':
		quit()

	elif choice=='yes' or choice=='y':

		while not rospy.is_shutdown():
			setpnt_pub.publish(quad_obj.quad_goal)
			rate.sleep()
			if quad_obj.state.mode=="OFFBOARD":
				if not quad_obj.state.armed==True:
					print "Arming..."
					rospy.wait_for_service('/chief/cmd/arming')
					arming_serv(True)
					rospy.sleep(2)


				print "Taking Off..."
				quad_obj.quad_goal.pose.position.x = quad_obj.vehicle.x
				quad_obj.quad_goal.pose.position.y = quad_obj.vehicle.y
				quad_obj.quad_goal.pose.position.z = 1

				for i in range(200):
					setpnt_pub.publish(quad_obj.quad_goal)
					rate.sleep()
				break
	print "mission start"
	mission()