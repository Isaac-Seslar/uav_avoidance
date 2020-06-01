#! /usr/bin/python

####################
# import libraries #
####################
import pandas as pd
import math
import rospy
import mavros
from mavros_msgs.msg import GlobalPositionTarget
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
# import mavros_msgs.msg
import std_msgs
from geometry_msgs.msg import PoseStamped, TwistStamped
# import geometry_msgs.msg
import tf 

###########################################################
# Read the .txt file and define each column to a variable #
###########################################################
quad_val = []
time     = []
x        = []
y        = []
z        = []
phi      = []
theta    = []
psi      = []

quad_val = pd.read_csv('~/uav_avoidance/src/uav_flight/src/Quadrotor.txt', sep=',')
time     = quad_val.time
x        = quad_val.x
y        = quad_val.y
z        = quad_val.z
phi      = quad_val.phi
theta    = quad_val.theta
psi      = quad_val.psi

# for p in len
x_write = x.to_csv('~/Desktop/quadx.txt')

# # print(len(time))
# print(len(x))
# ###############
# Definitions #
###############

# Define a loop and Menu #
##########################
def menu():
    print("Please choose from the following: \n 1: Arm the Quadrotor \n 2: Set Guided Mode \n 3: Takeoff \n 4: Go to Waypoints")


def myLoop():
    choice = '1'
    while not rospy.is_shutdown() and choice in ['1', '2', '3' ]:
        menu()
        choice = raw_input("Enter your input: ");
        if (choice == '1'):
            setArm()
        elif (choice == '2'):
            setGuidedMode()
        elif (choice == '3'):
            setTakeoffMode()
        elif (choice == '4'):
            # waypoint(time, x, y, z)
            break

# Arms the Quadrotor #
######################
def setArm():
    rospy.wait_for_service('mavros/cmd/arming')
    try:
        arming_serv = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
        arming_serv(True)  ##### maybe use (value='True') or (value=True)
        print("Arming...")
    except rospy.ServiceException as e:
        print("Arming Failed")


# Set mode of Quadrotor #
#########################
def setGuidedMode():
    rospy.wait_for_service('mavros/set_mode')
    try:
        flight_mode = rospy.ServiceProxy('mavros/set_mode', SetMode)
        change_flight_mode = flight_mode(custom_mode='OFFBOARD')
    except rospy.ServiceException as e:
        print("set_mode failed and OFFBOARD mode could not be set")

# Command Quadrotor to takeoff #
################################
def setTakeoffMode():
    rospy.wait_for_service('mavros/cmd/takeoff')
    try:
        takeoffService = rospy.ServiceProxy('mavros/cmd/takeoff', CommandTOL)
        takeoffService(altitude = 2.5, latitude = 0, longitude = 0, min_pitch = 0, yaw = 0)
    except rospy.ServiceException as e:
        print("Takeoff call failed")

# Takeoff and define setpoints for quadrotor #
##############################################
def waypoint(time, x, y, z):
    global i

    
    # setpoint = rospy.Publisher('mavros/plugins/setpoint_position', GlobalPositionTarget)

    setpoint_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
    # if time == time[0]:
    #     time = time[i]
    # else:
    #     time = time[i] - time[i-1]

    time = time
    msg = PoseStamped()
    msg.header.stamp = rospy.Time.now()

    while not rospy.is_shutdown():

        for i in [0, len(time)]:
            print(x[i])
            msg.pose.position.x = x[i]
            msg.pose.position.y = y[i]
            msg.pose.position.z = z[i]

            setpoint_pub.publish
            rospy.sleep(1)


#####################
# Begin Python Node #
#####################

rospy.init_node('quad_pos_read_code', anonymous=False)
# rospy.wait_for_service('mavros/src/plugins/setpoint_position')
# rospy.Subscriber("/mavros/state", NavSatFix, state_cb )
setpoint_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
velocity_pub = rospy.Publisher('mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)


while not rospy.is_shutdown():

    myLoop()
    print("here1")

    # while not rospy.is_shutdown():

    for i in range(len(time)):
        
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        print(x[i])
        msg.pose.position.x = x[i]
        msg.pose.position.y = y[i]
        msg.pose.position.z = z[i]

        setpoint_pub.publish
        rospy.sleep(.25)
