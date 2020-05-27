#! /usr/bin/python

####################
# import libraries #
####################
import pandas as pd
import math
import rospy
import mavros
import mavros_msgs
import std_msgs
import geometry_msgs
import tf 

###########################################################
# Read the .txt file and define each column to a variable #
###########################################################
quad_val = pd.read_csv('~/uav_avoidance/src/uav_flight/src/uav_flight/Quadrotor.txt', sep=',')
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
def myLoop():
    x = '1'
    while not rospy.is_shutdown() and x in ['1', '2', '3' ]:
        menu()
        x = raw_input("Enter your input: ");
        if (x == '1'):
            setArm()
        elif (x == '2'):
            setGuidedMode()
        elif (x == '3'):
            setTakeoffMode()
        else:
            break

# Arms the Quadrotor #
######################
def setArm():
    rospy.wait_for_service('mavros/cmd/arming')
    try:
        arming_serv = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
        arming_serv(True)
        print("Arming...")
    except rospy.ServiceException as e:
        print("Arming Failed")


# Set mode of Quadrotor #
#########################
def setGuidedMode():
    rospy.wait_for_service('/mavros/set_mode')
    try:
        flight_mode = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
        change_flight_mode = flight_mode(custom_mode='OFFBOARD')
    except rospy.ServiceException as e:
        print("set_mode failed and OFFBOARD mode could not be set")

# Command Quadrotor to takeoff #
################################
def setTakeoffMode():
    rospy.wait_for_service('mavros/cmd/takeoff')
    try:
        takeoffService = rospy.ServiceProxy('/mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
        takeoffService(altitude = z[2], latitude = 0, longitude = 0, min_pitch = 0, yaw = 0)
    except rospy.ServiceException as e:
        print("Takeoff call failed")

# Takeoff and define setpoints for quadrotor #
##############################################
def waypoint(time, x, y, z):
    rospy.wait_for_service('mavros/mavros/src/plugins/setpoint_position')
    setpoint = rospy.Publisher('~/uav_avoidance/src/uav_flight/launch/mavros/mavros/src', mavros_msgs.msg.GlobalPositionTarget)
    
    # if time == time[0]:
    #     time = time[i]
    # else:
    #     time = time[i] - time[i-1]
    time = time[i]
    msg.pose.pose.position.x = x[i]
    mavros_msgs.pose.position.y = y[i]
    mavros_msgs.pose.position.y= z[i]

while not rospy.is_shutdown():
    for i in [1, len(time)]:

        waypoint(time, x, y, z)

        if i == 1:
            print("Traveling to waypoints...")
        else:
            pass