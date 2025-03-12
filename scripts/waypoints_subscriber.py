#!/usr/bin/env python3
import rospy
import json
from std_msgs.msg import String

def callback(data):
    waypoints = json.loads(data.data)  # JSON 형태의 데이터를 로드
    rospy.loginfo("🚗 로봇이 따라갈 경로: %s", waypoints)

def listener():
    rospy.init_node("waypoints_subscriber", anonymous=True)
    rospy.Subscriber("waypoints", String, callback)
    rospy.spin()

if __name__ == "__main__":
    listener()
