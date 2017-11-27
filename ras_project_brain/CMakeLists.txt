cmake_minimum_required(VERSION 2.8.3)
project(ras_project_brain)

find_package(catkin REQUIRED COMPONENTS
      geometry_msgs
      roscpp
      rospy
      tf
      std_msgs
      message_generation)

add_service_files(
  FILES
  SetGoalPoint.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
)

catkin_package(CATKIN_DEPENDS roscpp rospy geometry_msgs std_msgs)
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(brain_node src/firstRun_node.cpp)
add_dependencies(brain_node ras_project_brain_generate_messages_cpp)
target_link_libraries(brain_node ${catkin_LIBRARIES})
