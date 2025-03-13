#!/usr/bin/env python3
import rospy
import json
from std_msgs.msg import String

def waypoints_callback(msg):
    waypoints = json.loads(msg.data)
    rospy.loginfo(f"📍 Waypoints 수신: {waypoints}")

    # 로봇이 waypoints를 따라가는 로직 추가 가능
    for point in waypoints:
        lat, lon = point['latitude'], point['longitude']
        rospy.loginfo(f"➡ 이동 좌표: 위도 {lat}, 경도 {lon}")

# ROS 노드 및 서브스크라이버 초기화
rospy.init_node('waypoint_subscriber', anonymous=True)
rospy.Subscriber('/waypoints', String, waypoints_callback)

rospy.spin()  # 노드 실행 유지
