# 변경사항

깃헙 명령어

git add .
git commit -m "커밋 메시지"
git push origin main

#3월 12일
GPS + Waypoint 기능 유지
Waypoint 일정거리이상(5m)로 다가가면 지나갔다는 판정 -> waypoint 마커 삭제
ROS 토픽 Publisher + Subscriber 추가

rosrun global_localization gps_server.py



##3월 13일
gps_publisher.py및 gps_server.py 추가

자바스크립트 수정
-목적지 검색 창 제거
-waypoint및 목적지 위도,경도 토픽으로 발행 기능 추가

gps_server.py로 먼저 웹 프롬포트 띄운 후에 gps_publisher.py 실행하여 gps로 좌표 받아오고 웹페이지에 목적지 찍으면 
카카오 네비 api를 사용하여 waypoint와 도착지 위도,경도 좌표 ROS topic으로 넘어옴 

명령어
rosrun global_localization gps_server.py
rosrun global_localization gps_publisher.py
rostopic echo waypoints


