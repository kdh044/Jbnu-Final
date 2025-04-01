cmake_minimum_required(VERSION 3.0.2)
project(global_localization)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Python 노드 설치 설정
catkin_install_python(
  PROGRAMS
    scripts/gps_server.py
    scripts/waypoints_subscriber.py
    scripts/waypoints_publisher.py
    scripts/gps_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# launch 파일 설치 설정
install(FILES
  launch/my_nodes.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
