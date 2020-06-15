#! /usr/bin/python

####################
# Import libraries #
####################

import math
from numpy import linspace
import rospy
import mavros
from mavros_msgs.msg import GlobalPositionTarget, State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
import std_msgs
from geometry_msgs.msg import PoseStamped, TwistStamped
import tf 
import sys, select, os, tty, termios
import imp

from RTA_gate import *
from com_func import *
from safe_cont import *
from perform_cont import *

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

# Defines the choices availible #
#################################
def menu():

	print "1: Arm, Takeoff, and Loiter at altitude of 3m \
	 \n2: Land \n3: Kill node"


# Arms quadrotor and commands it to takeoff and loiter at z = 3 #
#################################################################
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
		char = 0

		
		while True:

			os.system('cls' if os.name == 'nt' else 'clear')
			print "1: Start Mission\n2: Abort"

			setpoint_pub.publish(pose_stamp)
			rospy.sleep(.25)
			char = getch()

			if char == "1":
				mission()
				char = 0
				break

			if char == "2":
				offb_set_mode = flight_mode(custom_mode="AUTO.LAND")
				break

	except rospy.ServiceException as e:
		print("Failed to set OFFBOARD mode")


# Generates waypoints for the quadrotor top follow #
####################################################
def mission():

	pose_stamp = PoseStamped()

	if abs(x) >= abs(y) and abs(x) >= abs(z):
		gcoor = abs(x) 
		x_lin = linspace(0, x, gcoor)
		y_lin = linspace(0, y, gcoor)
		z_lin = linspace(0, z, gcoor)

	if abs(y) > abs(x) and abs(y) >= abs(z):
		gcoor = abs(y)
		x_lin = linspace(0, x, gcoor)
		y_lin = linspace(0, y, gcoor)
		z_lin = linspace(0, z, gcoor)

	if abs(z) > abs(x) and abs(z) > abs(y):
		gcoor = z
		x_lin = linspace(0, x, gcoor)
		y_lin = linspace(0, y, gcoor)
		z_lin = linspace(0, z, gcoor)

	for i in range(gcoor):
		print(x_lin[i])
		print(y_lin[i])
		print(z_lin[i])

		pose_stamp.pose.position.x = x_lin[i]
		pose_stamp.pose.position.y = y_lin[i]
		pose_stamp.pose.position.z = z_lin[i]

		setpoint_pub.publish(pose_stamp)
		rospy.sleep(.15)

		if i == len(range(gcoor)):
		# 	print("Returning to origin...")

		# 	for orig_ret in range(gcoor):
		# 		pose_stamp.pose.position.x = 0
		# 		pose_stamp.pose.position.y = 0
		# 		pose_stamp.pose.position.z = 3

			break


# Allows for enough time to cleanly kill the node #
###################################################
def kill():
	print "\n*Cough cough*"
	rospy.sleep(.6)
	print "\nI'm..."
	rospy.sleep(.5)
	print "\ndying"
	rospy.sleep(.5)
	rospy.signal_shutdown("Time to sleep!")
	os.system('cls' if os.name == 'nt' else 'clear')


#########
# Start #
#########

if __name__ == '__main__':

	global x, y, z

	rospy.init_node('waypoint_code', anonymous=False)

	# current_state  = mavros_msgs.State.C

	state_sub	   = rospy.Subscriber("mavros/state", State)

	setpoint_pub   = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
	velocity_pub   = rospy.Publisher('mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)

	arming_serv    = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
	flight_mode    = rospy.ServiceProxy('mavros/set_mode', SetMode)
	takeoffService = rospy.ServiceProxy('mavros/cmd/takeoff', CommandTOL)

	rate = rospy.Rate(20.0)

	choice = '1'

	while not rospy.is_shutdown() and choice in ['1', '2', '3']:
		os.system('cls' if os.name == 'nt' else 'clear')
		menu()
		choice = raw_input("Enter your input: ");
		x = 0
		y = 0
		z = 0

		if choice == '1':
			print "Enter destination coordinates: \n"
			x = int(raw_input('x = '))
			y = int(raw_input('y = '))
			z = int(raw_input('z = '))

			if z <= 0:
				print("z must be greater than 0\nAutomatically setting z = 1")
				rospy.sleep(2)
				z = 1

			arm_takeoff()

		if choice == '2':
			offb_set_mode = flight_mode(custom_mode="AUTO.LAND")

		if choice == '3':
			kill()
			break