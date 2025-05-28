cmake_minimum_required(VERSION 2.8.3)
project(car_demo)

set (PRIUSCUP_MAJOR_VERSION 0)
set (PRIUSCUP_MINOR_VERSION 1)
set (PRIUSCUP_PATCH_VERSION 0)
set (PRIUSCUP_VERSION_FULL ${PRIUSCUP_MAJOR_VERSION}.${PRIUSCUP_MINOR_VERSION}.${PRIUSCUP_PATCH_VERSION})

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  prius_msgs
)

# ✅ Gazebo 버전 명시 제거
find_package(gazebo REQUIRED)

# ✅ Ignition은 사용하지 않으면 제거 (Gazebo Classic이라면 거의 사용 안 함)
# find_package(ignition-msgs1 REQUIRED)  ← 이 줄 주석 처리 또는 삭제

catkin_package(
  CATKIN_DEPENDS gazebo_ros prius_msgs
)

# ✅ include 디렉토리에 있는 헤더들과 Gazebo include 포함
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

# ✅ 플러그인 빌드
add_library(PriusHybridPlugin SHARED plugins/PriusHybridPlugin.cc)

target_link_libraries(PriusHybridPlugin
  ${GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES}
)

# ✅ 빌드 순서 보장
add_dependencies(PriusHybridPlugin ${catkin_EXPORTED_TARGETS})

# ✅ 파이썬 노드 설치
catkin_install_python(PROGRAMS nodes/joystick_translator
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# ✅ 플러그인 설치
install(TARGETS PriusHybridPlugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# ✅ 모델 및 월드 파일 설치
install(DIRECTORY models DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY worlds DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
