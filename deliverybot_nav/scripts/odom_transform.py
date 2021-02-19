#!/usr/bin/env python
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

pub = rospy.Publisher('/dbot/odom', Odometry, queue_size=10)

def cb(msg):
    new_odom = Odometry()
    new_odom.header = msg.header
    new_odom.header.frame_id = "/dbot/odom"
    new_odom.pose = msg.pose
    new_odom.pose.pose.position.z = 0.0
    new_odom.twist = msg.twist
    new_odom.child_frame_id = msg.child_frame_id
    pub.publish(new_odom)

if __name__ == "__main__":
    rospy.init_node("odom_changer")
    rospy.Subscriber('/dbot/odom', Odometry, cb, queue_size=10)
    rospy.spin()
