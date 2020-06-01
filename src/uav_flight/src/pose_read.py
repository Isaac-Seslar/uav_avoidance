#! /usr/bin/python

####################
# Import libraries #
####################
import pandas as pd
import math
import rospy
import mavros
from mavros_msgs.msg import GlobalPositionTarget, State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
import std_msgs
from geometry_msgs.msg import PoseStamped, TwistStamped
import tf 
import sys, select, os

###########################################################
# Read the .txt file and define each column to a variable #
###########################################################
quad_val = pd.read_csv('~/uav_avoidance/src/uav_flight/src/Quadrotor.txt', sep=',')
time     = quad_val.time
x        = quad_val.x
y        = quad_val.y
z        = quad_val.z
phi      = quad_val.phi
theta    = quad_val.theta
psi      = quad_val.psi

###############
# Definitions #
###############

# Define a loop and Menu #
##########################
def menu():
	print "Please choose from the following: \n 1: Arm, Takeoff, and Loiter"


def myLoop():
	choice = '1'
	while not rospy.is_shutdown() and choice in ['1']:
		menu()
		choice = raw_input("Enter your input: ");
		if (choice == '1'):
			arm_takeoff()

		# if (choice =='2'):


def arm_takeoff():

	rospy.wait_for_service('mavros/set_mode')
	rospy.wait_for_service('mavros/cmd/arming')

	pose_stamp = PoseStamped()

	pose_stamp.pose.position.x = 0
	pose_stamp.pose.position.y = 0
	pose_stamp.pose.position.z = 3


	for i in range(20):
		setpoint_pub.publish(pose_stamp)
		rate.sleep()

	try:
 
		last_request = rospy.Time.now()

		offb_set_mode = flight_mode(custom_mode="OFFBOARD")
		arm_client = arming_serv(True)
		print "Take off and hover initiated"
		# rospy.sleep(1)

		
		while True:

			os.system('cls' if os.name == 'nt' else 'clear')
			# print "Loitering. \nPlease choose from the following: \nEnter: Fly mission\n 2: Land Quadrotor"
			print "Press Enter to start the mission"

			setpoint_pub.publish(pose_stamp)
			rospy.sleep(.25)

			if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
				line = raw_input()
				mission()

			# if n == 2:
			# 	offb_set_mode = flight_mode(custom_mode="AUTO.LAND")


	except rospy.ServiceException as e:
		print("Failed to set OFFBOARD mode")


def mission():

	pose_stamp = PoseStamped()

	for i in range(len(time)):

		pose_stamp.pose.position.x = x[i]
		pose_stamp.pose.position.y = y[i]
		pose_stamp.pose.position.z = z[i]

		quaternion = tf.transformations.quaternion_from_euler(phi[i], theta[i], psi[i])

		pose_stamp.pose.orientation.x = quaternion[0]
		pose_stamp.pose.orientation.y = quaternion[1]	
		pose_stamp.pose.orientation.z = quaternion[2]
		pose_stamp.pose.orientation.w = quaternion[3]

		setpoint_pub.publish(pose_stamp)
		rate.sleep()

		if i == len(time):
			print("Returning to origin...")



# rospy.wait_for_service('mavros/cmd/arming')
# rospy.wait_for_service('mavros/cmd/takeoff')
# rospy.wait_for_service('mavros/set_mode')

if __name__ == '__main__':

	rospy.init_node('quad_pos_read_code', anonymous=False)

	# current_state  = mavros_msgs.State.C

	state_sub	   = rospy.Subscriber("mavros/state", State)

	setpoint_pub   = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
	velocity_pub   = rospy.Publisher('mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)

	arming_serv    = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
	flight_mode    = rospy.ServiceProxy('mavros/set_mode', SetMode)
	takeoffService = rospy.ServiceProxy('mavros/cmd/takeoff', CommandTOL)

	rate = rospy.Rate(20.0)


	while not rospy.is_shutdown():

		myLoop()
