cmake_minimum_required(VERSION 2.8.3)
project(ras_project_tf_listener)

find_package(catkin REQUIRED COMPONENTS
      geometry_msgs
      roscpp
      rospy
      std_msgs
      tf)

catkin_package(CATKIN_DEPENDS roscpp rospy geometry_msgs tf)
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(ras_project_tf_listener src/tf_listener.cpp)
target_link_libraries(ras_project_tf_listener ${catkin_LIBRARIES})
add_dependencies(ras_project_tf_listener ras_proj_tf_listener_cpp)
