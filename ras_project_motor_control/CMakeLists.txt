cmake_minimum_required(VERSION 2.8.3)
project(ras_project_motor_controller)
add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS 
	roscpp
	rospy 
	geometry_msgs
	phidgets
)

catkin_package(LIBRARIES 
	motor_controller 
	CATKIN_DEPENDS 
	roscpp 
	rospy
	geometry_msgs
	phidgets
)
include_directories(${catkin_INCLUDE_DIRS}
)

add_executable(motor_controller src/motor_controller.cpp)
add_executable(motor_teleop src/motor_teleop.cpp)
target_link_libraries(motor_controller ${catkin_LIBRARIES})
target_link_libraries(motor_teleop ${catkin_LIBRARIES})

add_executable(motor_mockup src/motor_mockup.cpp)
target_link_libraries(motor_mockup ${catkin_LIBRARIES})
