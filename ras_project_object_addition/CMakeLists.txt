cmake_minimum_required(VERSION 2.8.3)
project(ras_project_object_addition)

find_package(catkin REQUIRED COMPONENTS 
	OpenCV
	roscpp
	rospy
        std_msgs
        tf
        sensor_msgs
	image_transport	
	cv_bridge
	pcl_conversions
	pcl_ros
	#message_generation
)

catkin_package(
LIBRARIES 
ras_project_object_addition
CATKIN_DEPENDS 
roscpp 
rospy 
tf 
std_msgs 
sensor_msgs 
image_transport
cv_bridge
pcl_conversions
pcl_ros 
#message_runtime
)

include_directories(${catkin_INCLUDE_DIRS})

#add_message_files(FILES RAS_Evidence.msg StringStamped.msg)

add_executable(object_addition src/object_addition.cpp)
target_link_libraries(object_addition ${catkin_LIBRARIES})
