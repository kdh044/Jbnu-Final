# 📌 전자공학부 4학년 종합설계 - Mapless 자율주행 프로젝트

## 👤 개인 작업 정리

### ✅ 완료
- Global Path Planning (패키지: `global_localization`, 3월 13일 완료)

### 🔄 진행 중
- Local Path Planning (DWA 기반 move_base 세팅)
- costmap 구성 및 RViz 테스트 환경 설정

---

## 🧠 주제:  
**지도 없이(Global Mapless) GPS + 센서 기반 자율주행 시스템 개발**  
GPS로 목적지를 설정하고, 센서(LiDAR, IMU 등)를 활용해 실시간 Local Path를 생성하며 주행하는 경량 자율주행 시스템 구현

---

## 📚 CONTENTS
- [Global Path Planning](#global-path-planning)
- [Semantic Segmentation](#semantic-segmentation)
- [Local Path Planning](#local-path-planning)

---

## 🌍 Global Path Planning

### 🔧 코드
패키지명: `global_localization`

### 📦 실행 명령어
```bash
rosrun global_localization gps_server.py
rosrun global_localization gps_publisher.py
rostopic echo waypoints
```

### 🛠 적용 형식
- 기존 JavaScript 코드 수정: 목적지 검색 창 제거
- Waypoint 및 목적지의 위도, 경도 값을 ROS 토픽으로 발행하는 기능 추가

### 🔁 실행 흐름
1. `gps_server.py` 실행 → 웹 프롬프트(Web UI) 띄움  
2. `gps_publisher.py` 실행 → GPS로 좌표 수신  
3. 웹페이지에서 목적지 선택 → 카카오 네비 API 활용  
4. Waypoint 및 목적지 좌표(위도, 경도) → ROS 토픽으로 전달  

---

## 🧠 Semantic Segmentation

### 📁 References
- LaserMix
- FRNet

### 🛠 적용 형식
- LaserMix의 teacher-student network 반지도학습 프레임을 이용
- teacher network에 선행학습된 checkpoints 적용
- 클래스 수: MMdetection 기준 20개 → 5개로 변경 (road, car, sidewalk, other-vehicle, unlabeled)

### ✅ 개선 사항
- `car`를 제외한 vehicle들을 `other-vehicle`로 묶고 향상된 학습법 적용
- IoU 점수: (67.45, 61.93) → 1.87%, 7.39% 향상
- mIoU: 논문 기준 74.69, 84.75 → 87.67%로 약 13%, 2.92% 향상

### 💻 코드
- `FRNet-LaserMix`

---

## 🚗 Local Path Planning

### 📌 Local Path Planner
- RL-DWA 기반

### 📍 Localization
명령어(0502 수정중)     
point cloud및 모델 불러오기까진 성공 ,dwa 진행중        
```bash
roslaunch husky_dwa husky_dwa_gazebo.launch        
roslaunch husky_dwa move_base.launch    
```
---

## 🗂️ 맵 정보 (최상위 디렉토리 기준)

- `mapping_01.pcd`: 6호관 ~ 7호관 사이
- `mapping_02.pcd`: 제2도서관 연구단지 쪽 길
- `mapping_03.pcd`: 농대 → 중도 정문
- `mapping_04.pcd`: 건지광장
- `mapping_05.pcd`: 7호관 → 2호관 → 4호관 → 3호관 → 공대입구
- `mapping_06.pcd`: 자연대 본관 ~ 3호관
- `mapping_07.pcd`: 공대 입구 → 진수당 한바퀴
- `mapping_08.pcd`: 진수당 주차장 → 법대 → 본부별관
- `mapping_09.pcd`: 법대 ~ 제2도서관 (loop closer 없음)
- `mapping_10.pcd`: 경상대 2호관 → 법대내리막
- `mapping_11.pcd`: 공대 공장동 → 후생관 → 인문대 → 실크로드 센터

