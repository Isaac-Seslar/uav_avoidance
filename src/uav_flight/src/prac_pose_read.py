#! /usr/bin/python
import rospy

from geometry_msgs.msg import PoseStamped, TwistStamped, PoseWithCovarianceStamped

rospy.init_node('practice', anonymous=False)

pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, queue_size = 10)

robotPose = PoseStamped()

while not rospy.is_shutdown():

	print robotPose
