#!/usr/bin/env python3

import rospy
import json
from std_msgs.msg import String

def callback(data):
    try:
        # /waypoints 토픽에서 수신한 메시지를 JSON으로 파싱
        msg_dict = json.loads(data.data)

        # waypoints 배열 파싱
        waypoints_list = msg_dict.get("waypoints", [])
        # 목적지 좌표 파싱
        destination = msg_dict.get("destination", None)

        # ROS 로그 출력
        rospy.loginfo(f"🔎 수신한 웨이포인트 개수: {len(waypoints_list)}")
        if destination:
            rospy.loginfo(f"📍 목적지: {destination}")
        else:
            rospy.loginfo("⚠️ 목적지 좌표 없음")

        # 여기서부터는 웨이포인트/목적지 데이터를 원하는 로직에 활용
        # 예: waypoints_list를 순회하며 주행 제어 등

    except Exception as e:
        rospy.logerr(f"JSON 파싱 오류: {e}")

def listener():
    rospy.init_node('waypoints_subscriber', anonymous=True)
    rospy.Subscriber("waypoints", String, callback)
    rospy.loginfo("🟢 waypoints_subscriber 노드 시작됨, /waypoints 토픽 구독 중...")
    rospy.spin()

if __name__ == '__main__':
    listener()
