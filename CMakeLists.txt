cmake_minimum_required(VERSION 2.8.3)
project(ras_map)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS rospy sensor_msgs geometry_msgs)

catkin_package(
  CATKIN_DEPENDS rospy sensor_msgs geometry_msgs
)

include_directories(
 ${catkin_INCLUDE_DIRS}
)

catkin_install_python(
  PROGRAMS scripts/map.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
