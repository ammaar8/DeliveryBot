#!/usr/bin/env python
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry



def cb(msg):
    br = tf2_ros.TransformBroadcaster()
    t = TransformStamped()
    t.header.stamp = msg.header.stamp
    t.header.frame_id = 'dbot/base_footprint'
    t.child_frame_id = 'dbot/base_link'
    t.transform.translation.x = 0.0
    t.transform.translation.y = 0.0
    t.transform.translation.z = 0.05 - msg.pose.pose.position.z
    t.transform.rotation.x = 0
    t.transform.rotation.y = 0
    t.transform.rotation.z = 0
    t.transform.rotation.w = 1
    br.sendTransform(t)


if __name__ == "__main__":
    rospy.init_node("odom_broadcaster")
    rospy.Subscriber('/dbot/odom_fake', Odometry, cb, queue_size=10)
    rospy.spin()
