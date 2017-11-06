cmake_minimum_required(VERSION 2.8.3)
project(ras_project_uarm)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  tf
  message_generation
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_service_files(
  FILES
  MoveArmCartesian.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs tf
)


add_executable(uarm_control_node src/uarm_control.cpp)
add_dependencies(uarm_control_node ras_project_uarm_generate_messages_cpp)
target_link_libraries(uarm_control_node
  ${catkin_LIBRARIES}
)

add_executable(uarm_transform_node src/uArm_transform.cpp)
target_link_libraries(uarm_transform_node
      ${catkin_LIBRARIES}
)
