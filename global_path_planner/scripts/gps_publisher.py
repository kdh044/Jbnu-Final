#!/usr/bin/env python3

import rospy
import serial
import json
import asyncio
import websockets
from std_msgs.msg import String
import threading
import signal
import sys

# 시리얼 포트 설정 (GPS 모듈 연결)
SERIAL_PORT = "/dev/ttyACM0"
BAUDRATE = 9600

# 최신 GPS 데이터 저장 (WebSocket + ROS 송신용)
latest_gps_data = None
data_lock = threading.Lock()

# ---------------------------
# 종료 핸들러 (Ctrl+C 처리)
# ---------------------------
def signal_handler(sig, frame):
    rospy.loginfo("🛑 종료 신호 감지됨! 프로그램 종료...")
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)  # Ctrl+C 감지

# ---------------------------
# ROS Publisher (gps_data)
# ---------------------------
def ros_publisher(pub):
    """ GPS 데이터를 ROS 토픽으로 전송 """
    global latest_gps_data

    try:
        ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
        rospy.loginfo("✅ 시리얼 포트 연결 성공: %s", SERIAL_PORT)
    except Exception as e:
        rospy.logerr("❌ 시리얼 포트 연결 실패: %s", e)
        return

    while not rospy.is_shutdown():
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()

            if line.startswith('$GPGGA'):  # GPGGA 메시지만 처리
                parts = line.split(',')
                if len(parts) >= 6 and parts[2] and parts[4]:
                    lat_raw, ns, lon_raw, ew = parts[2], parts[3], parts[4], parts[5]

                    # 위도 변환 (소수점 6자리까지)
                    latitude = round(float(lat_raw[:2]) + float(lat_raw[2:]) / 60.0, 6)
                    if ns.upper() == 'S':
                        latitude = -latitude

                    # 경도 변환 (소수점 6자리까지)
                    longitude = round(float(lon_raw[:3]) + float(lon_raw[3:]) / 60.0, 6)
                    if ew.upper() == 'W':
                        longitude = -longitude

                    gps_data = json.dumps({"latitude": latitude, "longitude": longitude})

                    # 최신 GPS 데이터 업데이트 (WebSocket & ROS 송신용)
                    with data_lock:
                        latest_gps_data = gps_data

                    # ROS 토픽으로 전송
                    pub.publish(gps_data)
                    rospy.loginfo("🚀 GPS 데이터 전송: %s", gps_data)
        except Exception as e:
            rospy.logerr("⚠ 센서 데이터 읽기 오류: %s", e)

# ---------------------------
# WebSocket 서버 (웹으로 실시간 전송)
# ---------------------------
async def send_gps_data(websocket, path):
    """ WebSocket을 통해 GPS 데이터를 실시간으로 웹에 전송 """
    global latest_gps_data
    while True:
        with data_lock:
            data_to_send = latest_gps_data
        if data_to_send:
            await websocket.send(data_to_send)
            rospy.loginfo("📡 WebSocket 전송: %s", data_to_send)
        await asyncio.sleep(1)  # 1초마다 전송

async def start_websocket_server():
    """ WebSocket 서버 실행 """
    async with websockets.serve(send_gps_data, "localhost", 8767):
        rospy.loginfo("🌐 WebSocket 서버 실행: ws://localhost:8767")
        await asyncio.Future()  # 무한 대기

# ---------------------------
# 메인 실행부
# ---------------------------
if __name__ == "__main__":
    # ✅ 메인 스레드에서 ROS 노드 초기화
    rospy.init_node("gps_publisher", anonymous=True)
    pub = rospy.Publisher("gps_data", String, queue_size=10)

    # ✅ ROS 노드 실행 (서브 스레드)
    ros_thread = threading.Thread(target=ros_publisher, args=(pub,), daemon=True)
    ros_thread.start()

    # ✅ WebSocket 서버 실행 (서브 스레드에서 asyncio 실행)
    websocket_thread = threading.Thread(target=lambda: asyncio.run(start_websocket_server()), daemon=True)
    websocket_thread.start()

    # ✅ Ctrl+C 감지
    rospy.spin()
