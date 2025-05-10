# 전자공학부 4학년 종합설계  
## Mapless 자율주행 프로젝트

---

## 개요

지도 없이 GPS 및 센서(LiDAR, IMU 등)를 활용해 실시간 Local Path를 생성하고 주행하는 경량 자율주행 시스템을 개발한다.  
Global path는 Kakao Navi API로부터 waypoint 좌표를 받아 생성하고, local path는 라이다 기반 도로 인식을 통해 DWA 로컬 플래너로 구현한다.

---

## 1. 개인 작업

### 1.1 Global Path Planning (완료)
- 패키지명: `global_localization`
- Kakao Navi API를 통해 목적지 검색
- Waypoint 및 목적지 위도/경도 좌표를 ROS 토픽으로 발행

#### 실행 흐름
1. `gps_server.py` 실행 → 웹 UI 생성  
2. `gps_publisher.py` 실행 → GPS 좌표 수신  
3. 웹에서 목적지 선택 → ROS 토픽 `/waypoints`로 전달

#### 명령어
```bash
rosrun global_localization gps_server.py
rosrun global_localization gps_publisher.py
rostopic echo waypoints
```

---

### 1.2 Local Path Planning (진행 중)
- DWA 기반 `move_base` 세팅
- costmap 구성, RViz에서 실시간 경로 시각화 완료
- 3D LiDAR를 2D LaserScan으로 변환하는 `pc2_to_scan` 설정 완료
- TF 충돌 해결, joint_state publisher 제거
- 실시간 주행 테스트 성공 (`Goal reached` 정상 확인)

#### 진행중인 작업
- DWA 파라미터 튜닝
- base_link–odom 프레임 간 TF 미스매칭 해결

#### 2025년 5월 10일 변경 사항
- OS1 LiDAR 센서: 64채널 → 32채널로 변경해 연산 부담 경감
- 센서 플러그인: CPU 기반(`libgazebo_ros_ouster_laser.so`) → GPU 가속 버전(`libgazebo_ros_ouster_gpu_laser.so`)으로 변경
- Gazebo 시뮬레이션 실시간성(Real-Time Factor): **0.14 → 1.0**로  향상

---

### 필수 패키지 설치 (Ubuntu 20.04 + ROS Noetic 기준)
```bash
sudo apt update && sudo apt install -y \
ros-noetic-husky-desktop \
ros-noetic-husky-simulator \
ros-noetic-ackermann-msgs \
ros-noetic-twist-mux \
ros-noetic-teleop-twist-keyboard \
ros-noetic-robot-localization \
ros-noetic-joint-state-publisher-gui \
ros-noetic-xacro \
ros-noetic-gazebo-ros-pkgs \
ros-noetic-gazebo-ros-control
```

#### 실행 명령어
```bash
roslaunch husky_dwa husky_dwa_gazebo.launch
roslaunch husky_dwa move_base.launch
```

---

## 2. 팀원 작업

### 2.1 FasterLIO 적용 (예정)
- 기존 ROS EKF localization 대체
- IMU + LiDAR 기반 경량화된 Odometry 추정
- 실환경에서도 고정밀 위치 추정 가능

---

### 2.2 LaserMix 기반 Semantic Segmentation
- 데이터셋: SemanticKITTI 기반
- 클래스 축소: road, car, sidewalk, other-vehicle, unlabeled
- 반지도학습(teacher-student network) 기반 모델 학습
- `car` 외의 차량을 `other-vehicle`로 통합

#### 성능 향상
- IoU: 67.45, 61.93 → +1.87%, +7.39%
- mIoU: 74.69 → 84.75 → 87.67% (최대 약 +13%)

#### 예정 기능
- 도로 영역 마스크를 Local Costmap Layer로 반영
- DWA 로컬 플래너가 도로 내에서만 주행하도록 유도

---

## 3. 맵 파일 목록 (`.pcd` 기준)

| 파일명          | 구간 요약 |
|-----------------|------------|
| mapping_01.pcd  | 6호관 ~ 7호관 사이 |
| mapping_02.pcd  | 제2도서관 → 연구단지 방향 |
| mapping_03.pcd  | 농대 → 중도 정문 |
| mapping_04.pcd  | 건지광장 일대 |
| mapping_05.pcd  | 7호관 → 2호관 → 4호관 → 3호관 → 공대입구 |
| mapping_06.pcd  | 자연대 본관 → 3호관 |
| mapping_07.pcd  | 공대 입구 → 진수당 순환 |
| mapping_08.pcd  | 진수당 주차장 → 법대 → 본부별관 |
| mapping_09.pcd  | 법대 ~ 제2도서관 (loop closer 없음) |
| mapping_10.pcd  | 경상대 2호관 → 법대 내리막 |
| mapping_11.pcd  | 공대 공장동 → 후생관 → 인문대 → 실크로드 센터 |
