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

add_executable(tf_listener_node src/tf_listener.cpp)
add_executable(object_tf_node src/objectPosition_transform.cpp)
target_link_libraries(tf_listener_node ${catkin_LIBRARIES})
target_link_libraries(object_tf_node ${catkin_LIBRARIES})
#add_dependencies(ras_project_tf_listener ras_proj_tf_listener_cpp)
