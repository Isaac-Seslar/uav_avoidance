#! /usr/bin/python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped, PoseWithCovarianceStamped

# This callback function will run every time a new message is received on the 
# /mavros/local_position/pose topic (see subscriber line below)
def local_pose_callback(msg):
    global robotPose   # May not need this, but probably good to do just in case
    robotPose = msg    # Store the message in a global variable for printing later

rospy.init_node('practice', anonymous=False)
robotPose = PoseStamped()

# Don't need the following line since local_position/pose is a ROS topic, not a service
#rospy.wait_for_service('/mavros/local_position/pose')

# The syntax for a rospy Subscriber is as follows (you were missing the
# callback, which is where all the real action happens)
pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped,
                            local_pose_callback, queue_size = 10)

while not rospy.is_shutdown():

    print robotPose

