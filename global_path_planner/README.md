# 🌍 Global Path Planner - `global_path_planner`

## ✅ 기능 요약
- GPS + Waypoint 기능 제공
- 목적지 입력 시, Kakao API를 활용하여 waypoint 리스트 생성
- Waypoint 도달(5m 이내) 시 자동 삭제
- ROS Topic 기반의 Publisher/Subscriber 구성

---

## 📅 작업 이력

### ✅ 3월 12일
- `gps_server.py`로 웹 프롬프트 제공
- Waypoint 접근 시 삭제 로직 구현
- ROS 토픽 구조 설계 (`/waypoints` 등)

### ✅ 3월 13일
- `gps_publisher.py` 및 기능 보완
- 자바스크립트 수정
  - 목적지 검색창 제거
  - waypoint 및 목적지 좌표를 ROS 토픽으로 전송

---

## 🚀 사용 방법

### Step 1. 실행 순서
```bash
rosrun global_path_planner gps_server.py
rosrun global_path_planner gps_publisher.py
rostopic echo /waypoints
```

👉 또는, 한 번에 실행하려면:
```bash
roslaunch global_path_planner my_nodes.launch
```
※ 단, 디버깅을 위해서는 위의 개별 실행을 권장함.

---

## 💡 기타 사항
- `/waypoints` 토픽을 통해 RViz에서 시각화 가능
- GPS 수신 기반이라 야외 주행을 위한 기반 기능임
