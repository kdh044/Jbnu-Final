import http.server
import socketserver
import threading
import webbrowser
import os
import time
import json
import asyncio
import websockets

PORT = 8000

# 간단한 HTTP 서버 (index.html 제공)
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

# WebSocket 서버 – 여기서는 예제용으로 고정 GPS 데이터(서울 좌표)를 전송함
async def send_gps_data(websocket, path):
    while True:
        gps_data = json.dumps({
            "latitude": 35.846013,
            "longitude": 127.134340
        })
        await websocket.send(gps_data)
        print("Sent GPS data:", gps_data)
        await asyncio.sleep(1)  # 1초 간격 전송

async def start_websocket_server():
    async with websockets.serve(send_gps_data, "localhost", 8765):
        print("WebSocket 서버 실행 중: ws://localhost:8765")
        await asyncio.Future()  # 무한 대기

if __name__ == '__main__':
    # HTTP 서버를 별도 스레드에서 실행
    threading.Thread(target=start_http_server, daemon=True).start()
    time.sleep(1)  # 서버 시작 대기
    open_browser()
    # WebSocket 서버 실행
    asyncio.run(start_websocket_server())
