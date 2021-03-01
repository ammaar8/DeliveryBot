#!/usr/bin/env python

import rospy
import sys
import tf_conversions
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def movebase_client(x, y, a):
	
	client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
	client.wait_for_server()

	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = "map"
	goal.target_pose.header.stamp = rospy.Time.now()

	goal.target_pose.pose.position.x = x
	goal.target_pose.pose.position.y = y
	goal.target_pose.pose.position.z = 0.0

	quat = tf_conversions.transformations.quaternion_from_euler(
		0.0,
		0.0,
		a
	)

	goal.target_pose.pose.orientation.x = quat[0]
	goal.target_pose.pose.orientation.y = quat[1]
	goal.target_pose.pose.orientation.z = quat[2]
	goal.target_pose.pose.orientation.w = quat[3]
	
	client.send_goal(goal)
	wait = client.wait_for_result()
	rospy.loginfo("Sent Goal")
	if not wait:
		rospy.logerr("Action server not available!")
		rospy.signal_shutdown("Action server not available!")
	else:
		return client.get_result()
		
if __name__ == "__main__":
	try:
		x = y = a = 0.0
		x = float(sys.argv[1])
		y = float(sys.argv[2])
		a = float(sys.argv[3])
		rospy.init_node("movebase_client_py")
		result = movebase_client(x, y, a)
		if result:
			rospy.loginfo("Goal execution done!")
	except rospy.ROSInterruptException:
		rospy.loginfo("Navigation test finised.")
		
		
