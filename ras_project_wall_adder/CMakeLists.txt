cmake_minimum_required(VERSION 2.8.3)
project(ras_project_wall_adder)
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


add_executable(wall_adder_node src/wall_adder.cpp)
target_link_libraries(wall_adder_node
  ${catkin_LIBRARIES}
)
