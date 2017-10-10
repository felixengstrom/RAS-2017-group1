cmake_minimum_required(VERSION 2.8.3)
project(ras_project_uArm)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(uArm_control_node src/uArm_control.cpp)

target_link_libraries(uArm_control_node
  ${catkin_LIBRARIES}
)

