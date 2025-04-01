#!/usr/bin/env python3
import rospy
import json
import asyncio
import websockets
from std_msgs.msg import String

# ROS 노드 및 퍼블리셔 초기화
rospy.init_node('waypoint_publisher', anonymous=True)
waypoints_pub = rospy.Publisher('/waypoints', String, queue_size=10)

async def handle_websocket(websocket, path):
    async for message in websocket:
        try:
            # WebSocket에서 받은 JSON 데이터
            data = json.loads(message)
            rospy.loginfo(f"📡 WebSocket 수신: {data}")

            # Vertexes(경로 좌표) 데이터가 있는 경우
            if "vertexes" in data:
                waypoints_pub.publish(json.dumps(data["vertexes"]))  # ROS 토픽 발행
                rospy.loginfo(f"🚀 Waypoints (Vertexes) ROS 발행 완료: {data['vertexes']}")

        except Exception as e:
            rospy.logerr(f"❌ WebSocket 데이터 처리 오류: {e}")

# WebSocket 서버 실행
start_server = websockets.serve(handle_websocket, "localhost", 8765)

async def main():
    async with start_server:
        await asyncio.Future()  # 서버 실행 유지

if __name__ == "__main__":
    try:
        asyncio.run(main())  # WebSocket 서버 실행
    except rospy.ROSInterruptException:
        pass
