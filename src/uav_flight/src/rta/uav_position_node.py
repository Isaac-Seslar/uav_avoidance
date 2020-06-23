#! /usr/bin/python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped, PoseWithCovarianceStamped

rospy.init_node('uav_position_node', anonymous=False)

def local_pose_callback(msg):
		global quad_pose
		quad_pose = msg    # Store the message in a global variable for printing later


if __name__ == '__main__':

	pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, local_pose_callback, queue_size=10)
	pose_pub = rospy.Publisher('UAV_pose', PoseStamped, queue_size=10)
	quad_pose = PoseStamped()
	rate = rospy.Rate(10) # 10hz

	while not rospy.is_shutdown():
		print quad_pose
		pose_pub.publish(quad_pose)
		rate.sleep()

	try:
		uav_position_node()
	except rospy.ROSInterruptException:
		pass