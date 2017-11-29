cmake_minimum_required(VERSION 2.8.3)
project(ras_project_error_detection)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  sensor_msgs
  tf
  message_generation
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

generate_messages(
    DEPENDENCIES
    std_msgs
    sensor_msgs
    geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs tf
  INCLUDE_DIRS
)


add_executable(error_detection_node src/error_detection.cpp)
target_link_libraries(error_detection_node
  ${catkin_LIBRARIES}
)
