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
import sys, select, os, tty, termios

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

def menu():
	print "1: Arm, Takeoff, and Loiter at altitude of 3m \
	 \n2: Return to home \n3: Land \
	 \n4: Kill node"

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

		if i == len(time) - 1:
			offb_set_mode = flight_mode(custom_mode="AUTO.LAND")
			break


def return_mission():
	pose_stamp=PoseStamped()

	# pose_stamp.pose.position.x = x[-1]
	# pose_stamp.pose.position.y = y[-1]
	# pose_stamp.pose.position.z = z[-1]


	for i in range(20):
		setpoint_pub.publish(pose_stamp)
		rate.sleep()

	offb_set_mode = flight_mode(custom_mode="OFFBOARD")
	arm_client = arming_serv(True)

	setpoint_pub.publish(pose_stamp)

	for r in reversed(range(len(time-1))):
		pose_stamp.pose.position.x = x[r]
		pose_stamp.pose.position.y = y[r]
		pose_stamp.pose.position.z = z[r]

		quaternion = tf.transformations.quaternion_from_euler(phi[r], theta[r], psi[r])

		pose_stamp.pose.orientation.x = quaternion[0]
		pose_stamp.pose.orientation.y = quaternion[1]	
		pose_stamp.pose.orientation.z = quaternion[2]
		pose_stamp.pose.orientation.w = quaternion[3]

		setpoint_pub.publish(pose_stamp)
		rate.sleep()


def kill():
	print "\n*Cough cough*"
	rospy.sleep(.6)
	print "\nI'm.."
	rospy.sleep(.5)
	print "\ndying"
	rospy.sleep(.5)
	rospy.signal_shutdown("Time to sleep!")
	os.system('cls' if os.name == 'nt' else 'clear')
	


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

		choice = '1'
		while not rospy.is_shutdown() and choice in ['1', '2', '3', '4']:
			os.system('cls' if os.name == 'nt' else 'clear')
			menu()
			choice = raw_input("Enter your input: ");

			if choice == '1':
				arm_takeoff()

			if choice == '2':
				return_mission()

			if choice == '3':
				offb_set_mode = flight_mode(custom_mode="AUTO.LAND")


			if choice == '4':
				kill()
				break