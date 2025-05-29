#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry

def odom_callback(msg):
    msg.header.frame_id = "odom"
    msg.child_frame_id = "body"  # 필요시 base_link
    pub.publish(msg)

rospy.init_node("odom_frame_relay")
pub = rospy.Publisher("/Odometry_fixed", Odometry, queue_size=10)
rospy.Subscriber("/Odometry", Odometry, odom_callback)
rospy.spin()
