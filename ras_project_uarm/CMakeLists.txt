cmake_minimum_required(VERSION 2.8.3)
project(ras_project_uarm)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
)

add_executable(uarm_control_node src/uarm_control.cpp)

target_link_libraries(uarm_control_node
  ${catkin_LIBRARIES}
)

