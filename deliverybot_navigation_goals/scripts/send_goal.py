#!/usr/bin/env python

import rospy
import sys
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def movebase_client(x, y, z):
	
	client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
	client.wait_for_server()

	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = "map"
	goal.target_pose.header.stamp = rospy.Time.now()
	goal.target_pose.pose.position.x = x
	goal.target_pose.pose.position.y = y
	goal.target_pose.pose.position.z = z
	goal.target_pose.pose.orientation.w = 1.0
	
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
		x = y = z = 0.0
		x = float(sys.argv[1])
		y = float(sys.argv[2])
		y = float(sys.argv[3])
		rospy.init_node("movebase_client_py")
		result = movebase_client(x, y, z)
		if result:
			rospy.loginfo("Goal execution done!")
	except rospy.ROSInterruptException:
		rospy.loginfo("Navigation test finised.")
		
		
