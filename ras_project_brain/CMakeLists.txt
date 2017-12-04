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
  PickUpObj.srv
)

add_message_files(
  FILES
  ObjPickup_Update.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
)

catkin_package(CATKIN_DEPENDS roscpp rospy geometry_msgs std_msgs message_runtime)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(brain_node src/firstRun_node.cpp)
add_dependencies(brain_node ras_project_brain_generate_messages_cpp)
target_link_libraries(brain_node ${catkin_LIBRARIES})

add_executable(uarm_state_node src/central_node.cpp)
add_dependencies(uarm_state_node ras_project_brain_generate_messages_cpp)
target_link_libraries(uarm_state_node ${catkin_LIBRARIES})
