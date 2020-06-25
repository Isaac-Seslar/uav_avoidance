#! /usr/bin/python

####################
# Import libraries #
####################

import math
import rospy
import mavros
import std_msgs
import tf 
import sys, select, os, tty, termios

from geometry_msgs.msg import PoseStamped, TwistStamped
from numpy import linspace, array
from mavros_msgs.msg import GlobalPositionTarget, State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL

# Personal def
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

# Tolerance function #
######################
def isclose(a, b, rel_tol=1e-09, abs_tol=0.0):
    return abs(a-b) <= max(rel_tol * max(abs(a), abs(b)), abs_tol)

# def local_pose_callback(msg):
#         global quad_pose
#         quad_pose = msg    # Store the message in a global variable for printing later


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
                # mission()
                char = 0
                break

            if char == "2":
                offb_set_mode = flight_mode(custom_mode="AUTO.LAND")
                break

    except rospy.ServiceException as e:
        print("Failed to set OFFBOARD mode")

def local_pose_callback(msg):
    global quad_pose_x, quad_pose_y, quad_pose_z
    quad_pose_x = msg.pose.position.x
    quad_pose_y = msg.pose.position.y
    quad_pose_z = msg.pose.position.z

# Generates waypoints for the quadrotor top follow #
####################################################
def mission():

    quad_pose_x = PoseStamped()
    quad_pose_y = PoseStamped()
    quad_pose_z = PoseStamped()

    pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped,
                                local_pose_callback, queue_size = 10)

    rate = rospy.Rate(10) #Loop operates at 10hz

    while not rospy.is_shutdown():

        # print quad_pose_x
        print quad_pose_x, quad_pose_y, quad_pose_z
        rate.sleep()





    # def callback(msg):
    #     global quad_pose_x, quad_pose_y, quad_pose_z
    #     # quad_pose_x = rospy.loginfo(data.pose.position.x)
    #     quad_pose_x = msg.pose.position.x
    #     quad_pose_y = msg.pose.position.y
    #     quad_pose_z = msg.pose.position.z
    #     print quad_pose_x, quad_pose_y, quad_pose_z 



    # pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, callback)

    # vehicle_x    = quad_pose_x
    # vehicle_y    = quad_pose_y
    # vehicle_xdot = 0.2 
    # vehicle_ydot = 0.2

    # rate = rospy.Rate(10) #Loop operates at 10hz

    # # Define final (x,y) coordinate
    # target = np.array([x,y])
    # x_targ = target[0]
    # y_targ = target[1]

    # quad_pose_x = PoseStamped()
    # quad_pose_y = PoseStamped()
    # quad_pose_z = PoseStamped()
    
    # # Set up tolerances for ending the loop
    # # x_tol = isclose(quad_pose_x, x_targ, abs_tol=0.25)
    # # y_tol = isclose(quad_pose_y, y_targ, abs_tol=0.25)
    # x_tol = 1
    # y_tol = 1



    # pose_stamp = PoseStamped()
    

    # while x_tol==1 and y_tol==1:

    #     wpt = RTA_Gate(vehicle_x, vehicle_y, vehicle_xdot, vehicle_ydot, 
    #         obstacle_list, target, current_mode, safe_complete, cur_wpt)

    #     vehicle_x    = quad_pose_x
    #     vehicle_y    = quad_pose_y
    #     vehicle_xdot = 0.2 
    #     vehicle_ydot = 0.2

    #     current_mode  = wpt[1]
    #     safe_complete = wpt[2]

    #     cur_wpt = wpt[0]

    #     quad_pose_x = cur_wpt[0] 
    #     quad_pose_y = cur_wpt[1]

    #     pose_stamp.pose.position.x = quad_pose_x
    #     pose_stamp.pose.position.y = quad_pose_y
    #     pose_stamp.pose.position.z = z

    #     x_tol = isclose(quad_pose_x, x_targ, abs_tol=0.25)
    #     y_tol = isclose(quad_pose_y, y_targ, abs_tol=0.25)

    #     setpoint_pub.publish(pose_stamp)

    #     rate.sleep()





    

# Responsible for switching between performance and safe controller #
#####################################################################
def RTA_Gate(vehicle_x, vehicle_y, vehicle_xdot, vehicle_ydot, 
    obstacle_list, target, current_mode, safe_complete, cur_wpt):

        #Parameters:
        obs_to_consider = 2 #Number of obstacles to consider in avoidance
        tol = 2 #Time (seconds) tolerance to avoid obstacle (3 means should always have a 3 second buffer between vehicle and obstacle)
        violation = 0 #Is a future collision detected?

        obs_total = np.size(obstacle_list, 1)
        num_obs = max(obs_to_consider, obs_total)  #Number of obstacles to care about (set to 2 right now, can adjust)

        obstacles = get_closest_obstacle(vehicle_x, vehicle_y, obstacle_list, num_obs)

        #Here is the basic boundary check based on current position and direction of vehicle. This will be modified with something fancier
        speed = math.sqrt(math.pow(vehicle_xdot,2) + math.pow(vehicle_ydot, 2))
        heading = math.atan2(vehicle_ydot, vehicle_xdot)

        if heading < 0:
            heading = 2*math.pi + heading

        d = max(speed*tol, 1) #distance separation required, should be at least 1 m


        for i in range(0, num_obs):

            rel_position_x = vehicle_x - obstacles[i][0]
            rel_position_y = vehicle_y - obstacles[i][1]
            obs_heading = math.atan2(-rel_position_y, -rel_position_x)
            if obs_heading < 0:
                obs_heading = 2*math.pi + obs_heading

            d_act = math.sqrt(math.pow(rel_position_x, 2) + math.pow(rel_position_y, 2))

            #Check if vehicle is currently maintaining a "tol" second buffer
            dist_check_1 = d_act - d

            #Obstacle is roughly in the current path of the vehicle
            if abs(heading - obs_heading) <= math.pi/6 and dist_check_1 <= 0:
                violation = 1
                collision_obstacle = i

            #Decide which waypoint to output
            if violation == 0 and current_mode == 0:
                wpt_performance = Performance_Controller(vehicle_x, vehicle_y, target)
                #This would be reinforcement learning controller. We can replace with something else for now.
                # wpt_out = wpt_performance # Assign wpt_ to a single variable that can be re-writen ########################### <--- Show Kendra
                # print(wpt_out)
                return wpt_performance, current_mode, safe_complete

            elif violation == 1 and current_mode == 0:
                #STart safe maneuver, reset current_mode, start timer
                current_mode = 1
                safe_complete = 0
                start_safe = 1
                wpt_safe_list = "global"
                Safe_Controller_Init(vehicle_x, vehicle_y, vehicle_xdot, vehicle_ydot, target, obstacle_list[i])
                print("middle")
                return wpt_safe_list[0],  current_mode, safe_complete

            elif current_mode == 1 and safe_complete == 0:
                #Continue to execute trajectory until completed
                start_safe = 0
                wpt_safe, safe_complete = Safe_Controller(vehicle_x, vehicle_y, target, cur_wpt)
                # wpt_out = wpt_safe # Assign wpt_ to a single variable that can be re-writen
                print("middle_bot")
                return wpt_safe, current_mode, safe_complete

            else:
                #Safe maneuver has been completed, hopefully we don't switch right back to violation so give buffer?
                current_mode = 0
                wpt_performance = Performance_Controller(vehicle_x, vehicle_y, target)
                # wpt_out = wpt_performance # Assign wpt_ to a single variable that can be re-writen
                print("def bottom")
                return wpt_performance, current_mode, safe_complete

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

    global x, y, z, wpt_out

    rospy.init_node('RTA_Gate', anonymous=False)

    # Subscribers
    state_sub      = rospy.Subscriber('/mavros/state', State)
    # pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, local_pose_callback, queue_size=10)

    # Publishers
    setpoint_pub   = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
    velocity_pub   = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
    local_position = rospy.Publisher('/mavros/local_position/pose', PoseStamped)

    # Services
    arming_serv    = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
    flight_mode    = rospy.ServiceProxy('/mavros/set_mode', SetMode)
    takeoffService = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)

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

            # arm_takeoff()
            mission()

        if choice == '2':
            offb_set_mode = flight_mode(custom_mode="AUTO.LAND")

        if choice == '3':
            kill()
            break