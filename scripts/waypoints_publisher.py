#!/usr/bin/env python3

import rospy
import json
from std_msgs.msg import String
import asyncio
import websockets

WAYPOINTS_TOPIC = "waypoints"

async def receive_waypoints():
    """웹에서 받은 경로 데이터를 ROS 토픽으로 전송"""
    pub = rospy.Publisher(WAYPOINTS_TOPIC, String, queue_size=10)
    rospy.init_node("waypoints_publisher", anonymous=True)

    async with websockets.serve(websocket_handler, "localhost", 8766):
        rospy.loginfo("🔗 WebSocket (Waypoints) 서버 실행 중: ws://localhost:8766")
        await asyncio.Future()  # 무한 대기

async def websocket_handler(websocket, path):
    """WebSocket 메시지를 수신하고 ROS로 퍼블리시"""
    pub = rospy.Publisher(WAYPOINTS_TOPIC, String, queue_size=10)
    async for message in websocket:
        try:
            waypoints = json.loads(message)  # JSON 파싱
            pub.publish(json.dumps(waypoints))
            rospy.loginfo(f"🗺️ 웹에서 받은 Waypoints: {waypoints}")
        except Exception as e:
            rospy.logerr(f"❌ Waypoints 처리 오류: {e}")

if __name__ == "__main__":
    asyncio.run(receive_waypoints())
