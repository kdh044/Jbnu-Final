#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import NavSatFix

def callback(msg):
    msg.position_covariance = [1e-4, 0, 0,
                               0, 1e-4, 0,
                               0, 0, 1e-4]
    msg.position_covariance_type = 2
    pub.publish(msg)

rospy.init_node("gps_covariance_relay")
pub = rospy.Publisher("/gps/fix_cov", NavSatFix, queue_size=10)
rospy.Subscriber("/gps/fix", NavSatFix, callback)
rospy.spin()
