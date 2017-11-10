cmake_minimum_required(VERSION 2.8.3)
project(ras_project_lidar)

## Compile as C++11, supported in ROS Kinetic and newer
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


add_executable(lidar_transform src/lidar_transform.cpp)
target_link_libraries(lidar_transform ${catkin_LIBRARIES})
add_executable(fakelidar_node src/fakelidar_node.cpp)
target_link_libraries(fakelidar_node ${catkin_LIBRARIES})
