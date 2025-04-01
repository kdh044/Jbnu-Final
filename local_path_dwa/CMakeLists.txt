cmake_minimum_required(VERSION 3.0.2)
project(local_path_dwa)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  move_base
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs move_base nav_msgs rospy sensor_msgs std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
