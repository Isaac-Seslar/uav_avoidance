#! /usr/bin/python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped, PointStamped, PoseArray

rospy.init_node('uav_position', anonymous=False)

def local_pose_callback(data):
		global quad_pose
		quad_pose = data    # Store the message in a global variable for printing later


if __name__ == '__main__':

	pose_pub = rospy.Publisher('UAV_pose', PoseStamped, queue_size=10)
	# x_pub    = rospy.Publisher('UAV_pose_x', PoseStamped, queue_size=10)
	# y_pub    = rospy.Publisher('UAV_pose_y', Point, queue_size=10)
	# z_pub    = rospy.Publisher('UAV_pose_z', Point, queue_size=10)

	pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, 
								local_pose_callback, queue_size=10)
	quad_pose = PoseStamped()

	# xyz = quad_pose.pose
	# print xyz

	rate = rospy.Rate(10) # 10hz

	while not rospy.is_shutdown():
		# print quad_pose
		pose_pub.publish(quad_pose)
		# x_pub.publish(xyz)
		print quad_pose
		rate.sleep()
		# print xyz

# 	# try:
# 	# 	uav_position_node()
# 	# except rospy.ROSInterruptException:
# 	# 	pass

