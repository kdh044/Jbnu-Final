import http.server
import socketserver
import threading
import webbrowser
import os
import time
import json
import asyncio
import websockets
import serial

PORT = 8000
SERIAL_PORT = '/dev/ttyACM0'  # 실제 센서가 연결된 포트
BAUDRATE = 9600

# 전역 변수에 최신 GPS 데이터를 저장 (쓰레드 안전하게 접근)
latest_gps_data = None
data_lock = threading.Lock()

# ---------------------------
# HTTP 서버 (index.html 제공)
# ---------------------------
class CustomHandler(http.server.SimpleHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/favicon.ico':
            self.send_response(204)
            self.end_headers()
            return
        return super().do_GET()

def start_http_server():
    with socketserver.TCPServer(("", PORT), CustomHandler) as httpd:
        print(f"HTTP 서버 실행 중: http://localhost:{PORT}")
        httpd.serve_forever()

def open_browser():
    url = f"http://localhost:{PORT}/index.html"
    webbrowser.open(url)

# ---------------------------
# 시리얼 포트를 통해 센서 데이터를 지속적으로 읽어오는 함수
# ---------------------------
def serial_reader():
    global latest_gps_data
    try:
        ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
        print("시리얼 포트 연결 성공:", SERIAL_PORT)
    except Exception as e:
        print("시리얼 포트 연결 실패:", e)
        return

    while True:
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            # NMEA GPGGA 메시지 예시: "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47"
            if line.startswith('$GPGGA'):
                parts = line.split(',')
                if len(parts) >= 6 and parts[2] and parts[4]:
                    lat_raw = parts[2]
                    ns = parts[3]
                    lon_raw = parts[4]
                    ew = parts[5]
                    # 위도 변환: "4807.038" -> 48 + 7.038/60
                    lat_deg = float(lat_raw[:2])
                    lat_min = float(lat_raw[2:])
                    latitude = lat_deg + lat_min / 60.0
                    if ns.upper() == 'S':
                        latitude = -latitude
                    # 경도 변환: "01131.000" -> 11 + 31.000/60
                    lon_deg = float(lon_raw[:3])
                    lon_min = float(lon_raw[3:])
                    longitude = lon_deg + lon_min / 60.0
                    if ew.upper() == 'W':
                        longitude = -longitude

                    # 최신 GPS 데이터를 전역 변수에 업데이트 (쓰레드 안전)
                    with data_lock:
                        latest_gps_data = {"latitude": latitude, "longitude": longitude}
                    print("읽은 센서 데이터:", latest_gps_data)
        except Exception as e:
            print("센서 데이터 읽기 오류:", e)
        time.sleep(0.1)  # CPU 과부하 방지를 위한 짧은 대기

# ---------------------------
# WebSocket 서버 – 최신 GPS 데이터를 전송
# ---------------------------
async def send_gps_data(websocket, path):
    global latest_gps_data
    while True:
        with data_lock:
            data_to_send = latest_gps_data
        if data_to_send is not None:
            gps_data = json.dumps(data_to_send)
        else:
            gps_data = json.dumps({"error": "센서 데이터 없음"})
        await websocket.send(gps_data)
        print("전송한 GPS 데이터:", gps_data)
        await asyncio.sleep(1)  # 1초마다 전송

async def start_websocket_server():
    async with websockets.serve(send_gps_data, "localhost", 8765):
        print("WebSocket 서버 실행 중: ws://localhost:8765")
        await asyncio.Future()  # 무한 대기

# ---------------------------
# 메인 실행부
# ---------------------------
if __name__ == '__main__':
    # HTTP 서버를 별도 스레드에서 실행
    threading.Thread(target=start_http_server, daemon=True).start()
    time.sleep(1)
    open_browser()

    # 시리얼 데이터 읽기를 위한 스레드 실행 (포트를 한 번 열어 지속적 읽기)
    threading.Thread(target=serial_reader, daemon=True).start()

    # WebSocket 서버 실행 (메인 스레드에서 asyncio 실행)
    asyncio.run(start_websocket_server())