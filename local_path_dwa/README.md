# 🤖 Local Path Planner (DWA 기반) - `local_path_dwa`

## ✅ 개요
- ROS `move_base` 패키지와 DWA Local Planner를 활용한 로컬 경로 계획
- 맵 없이(Local) 주행 가능한 설정 구성
- TurtleBot3 + Gazebo 시뮬레이션 기반 실험

---

## 📂 구성 파일

- `config/`: DWA Planner 및 costmap 관련 YAML 설정
- `launch/move_base.launch`: 전체 로컬 플래너 실행용 launch 파일

---

## 🚀 실행 방법

### Step 1. Gazebo 환경 실행
```bash
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

### Step 2. 상태 퍼블리셔 실행 (TF 생성을 위함)
```bash
roslaunch turtlebot3_bringup turtlebot3_model.launch
```

### Step 3. DWA 로컬 플래너 실행
```bash
roslaunch local_path_dwa move_base.launch
```

---

## 💡 주의사항
- `base_link ↔ odom` TF 미존재 시, move_base가 작동하지 않음
- LaserScan 토픽(`/scan`)과 TF 구조가 제대로 연결되어야 함
