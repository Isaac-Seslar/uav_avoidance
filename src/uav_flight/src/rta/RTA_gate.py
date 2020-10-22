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

from geometry_msgs.msg import PoseStamped, TwistStamped, TransformStamped, Point, Vector3
from numpy import linspace, array
from mavros_msgs.msg import GlobalPositionTarget, State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL

# Personal def
from Common_Functions import *
from Safe_Controller import *
from Performance_Controller import *

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
def mission(x,y,z,rate):
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

    # Define final (x,y) coordinate
    target = np.array([x,y])
    x_targ = target[0]
    y_targ = target[1]

    # Set up tolerances for ending the loop
    x_tol = isclose(quad_obj.float_x, x_targ, abs_tol=0.1)
    y_tol = isclose(quad_obj.float_y, y_targ, abs_tol=0.1)

    while x_tol==False and y_tol==False:

        wpt = RTA_Gate(quad_obj.float_x,quad_obj.float_y,quad_obj.float_xdot,
        	quad_obj.float_ydot,obstacle_list,target,current_mode,safe_complete,cur_wpt)

        # vehicle_x    = quad_pose_x
        # vehicle_y    = quad_pose_y
        # vehicle_xdot = 0.2 
        # vehicle_ydot = 0.2

        current_mode  = wpt[1]
        safe_complete = wpt[2]

        cur_wpt = wpt[0]

        x_rta = cur_wpt[0] 
        y_rta = cur_wpt[1]

        quad_obj.quad_goal.pose.position.x = x_rta
        quad_obj.quad_goal.pose.position.y = y_rta
        quad_obj.quad_goal.pose.position.z = z

        x_tol = isclose(quad_obj.float_x, x_targ, abs_tol=0.1)
        y_tol = isclose(quad_obj.float_y, y_targ, abs_tol=0.1)
        # print x_rta, quad_obj.float_x, x_targ, x_tol 
        # print x_tol 

        setpnt_pub.publish(quad_obj.quad_goal)

        rate.sleep()

# Responsible for switching between performance and safe controller #
def RTA_Gate(vehicle_x, vehicle_y, vehicle_xdot, vehicle_ydot, obstacle_list,
			 target, current_mode, safe_complete, cur_wpt):
        global wpt_safe_list  #Parameters:

        obs_to_consider = 2 #Number of obstacles to consider in avoidance
        obs_radius = 0.5 #Let's assume each obstacle is circular with radius obs_radius
        tol = 0.5 #Time (seconds) tolerance to avoid obstacle (3 means should always have a 3 second buffer between vehicle and obstacle)
        violation = 0 #Is a future collision detected?

        obs_total = np.size(obstacle_list, 1)
        num_obs = max(obs_to_consider, obs_total)  #Number of obstacles to care about (set to 2 right now, can adjust)

        obstacles = get_closest_obstacle(vehicle_x, vehicle_y, obstacle_list, num_obs)

        #Here is the basic boundary check based on current position and direction of vehicle. This will be modified with something fancier
        speed = math.sqrt(math.pow(vehicle_xdot,2) + math.pow(vehicle_ydot, 2))
        heading = math.atan2(vehicle_ydot, vehicle_xdot)

        if heading < 0:
            heading = 2*math.pi + heading
        print('heading')
        print(heading)
        d = max(speed*tol, 0.5) + obs_radius #distance separation required, should be at least 0.5 m

        print('min distance')
        print(d)

        if current_mode == 0: #Look for possibel collision
            for i in range(0, num_obs):
                print(' ')
                print('obstacle:')
                print(obstacles[i])

                rel_position_x = obstacles[i][0] - vehicle_x
                rel_position_y = obstacles[i][1] - vehicle_y
                obs_heading = math.atan2(rel_position_y, rel_position_x)
                if obs_heading < 0:
                    obs_heading = 2*math.pi + obs_heading

                print('obstacle heading')
                print(obs_heading)
                d_act = math.sqrt(math.pow(rel_position_x, 2) + math.pow(rel_position_y, 2)) #distance from center of obstacle

                print('actual distance')
                print(d_act)
                #Check if vehicle is currently maintaining a "tol" second buffer
                dist_check_1 = d_act - d
                #print(heading)
                #print(obs_heading)
                #print(dist_check_1)

                #Obstacle is roughly in the current path of the vehicle
                if abs(heading - obs_heading) <= math.pi/4 and dist_check_1 <= 0:
                    violation = 1
                    collision_obstacle = i
                    break

            print(' ')
            #Decide which waypoint to output
            if violation == 0:
                print('no violation detected')
                wpt_performance = Performance_Controller(vehicle_x, vehicle_y, target)
                #This would be reinforcement learning controller. We can replace with something else for now.
                return wpt_performance, current_mode, safe_complete

            elif violation == 1:
                #STart safe maneuver, reset current_mode, start timer
                print('started safe mode')
                current_mode = 1
                safe_complete = 0
                start_safe = 1

                wpt_safe_list = Safe_Controller_Init(vehicle_x, vehicle_y, vehicle_xdot, vehicle_ydot, target, obstacle_list[i])
                print(wpt_safe_list)
                #plt.plot(wpt_safe_list[:, 0], wpt_safe_list[:, 1])
                #plt.show()
                return wpt_safe_list[0],  current_mode, safe_complete

        elif current_mode == 1 and safe_complete == 0:
            print('continuing in safe mode')
            #Continue to execute trajectory until completed
            start_safe = 0
            wpt_safe, safe_complete = Safe_Controller(vehicle_x, vehicle_y, target, cur_wpt, wpt_safe_list)

            return wpt_safe, current_mode, safe_complete

        else:
            print('switching back to normal mode')
            #Safe maneuver has been completed, hopefully we don't switch right back to violation so give buffer?
            current_mode = 0
            wpt_performance = Performance_Controller(vehicle_x, vehicle_y, target)

            return wpt_performance, current_mode, safe_complete##################################################################


#########
# Start #
#########

if __name__ == '__main__':

	rospy.init_node('RTA_Gate', anonymous=False)

	rate = rospy.Rate(50) 

	# Time setup
	now       = rospy.Time.now()
	zero_time = rospy.Time()

	# Subscribers
	state_sub   = rospy.Subscriber('/mavros1/state', State, state_callback, queue_size=10)
	quad_pose_sub  = rospy.Subscriber('/mavros1/local_position/pose', PoseStamped,
	                            pose_callback, queue_size=10)
	velo_sub       = rospy.Subscriber('/mavros1/local_position/velocity', TwistStamped,
	                            velocity_callback, queue_size=10)

	obs_l_pose_sub = rospy.Subscriber('/vicon/obs_l/obs_l', TransformStamped,
	                            obs_l_callback, queue_size=10)
	obs_s_pose_sub = rospy.Subscriber('/vicon/obs_s/obs_s', TransformStamped,
	                            obs_s_callback, queue_size=10)


	# Publishers
	setpnt_pub     = rospy.Publisher('/mavros1/setpoint_position/local', PoseStamped, queue_size=10)


	# velocity_pub   = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
	# local_position = rospy.Publisher('/mavros/local_position/pose', PoseStamped)

	# Services
	setmod_serv  = rospy.ServiceProxy('/mavros1/set_mode', SetMode)
	arming_serv  = rospy.ServiceProxy('/mavros1/cmd/arming', CommandBool)
	takeoff_serv = rospy.ServiceProxy('/mavros1/cmd/takeoff', CommandTOL)
	landing_serv = rospy.ServiceProxy('/mavros1/cmd/land', CommandTOL)

	print "Quick Test?(x=1.8, y=1.4, z=1)\n"

	choice = raw_input('yes or no: ')

	if choice=='yes' or choice=='y':
		x=1.8
		y=1.4
		z=1

	elif choice=='no' or choice=='n':
		print "Enter destination coordinates: \n -1.3<x<1.8\n -1.5<y<1.4\n0<z<2"
		x = float(raw_input('x = '))
		y = float(raw_input('y = '))
		z = float(raw_input('z = '))

		if x>1.8 or x<-1.3:
			print "ERROR: x waypoint outside test area"
			quit()

		if y>1.4 or y<-1.5:
			print "ERROR: y waypoint outside test area"
			quit()

		if z>2 or z<0:
			print "ERROR: z waypoint outside test area"
			quit()

	while not rospy.is_shutdown():
		setpnt_pub.publish(quad_obj.quad_goal)
		rate.sleep()
		if quad_obj.state.mode=="OFFBOARD":
			if not quad_obj.state.armed==True:
				print "Arming..."
				rospy.wait_for_service('/mavros1/cmd/arming')
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

	mission(x,y,z,rate)