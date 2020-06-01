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

n = 1

import sys, select, os

i = 0
while True:
    os.system('cls' if os.name == 'nt' else 'clear')
    print "I'm doing stuff. Press Enter to stop me!"
    print i
    if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
        line = raw_input()
        break
    i += 1
