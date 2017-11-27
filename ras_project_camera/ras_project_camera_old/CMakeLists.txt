cmake_minimum_required(VERSION 2.8.3)
project(ras_project_camera)

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
)

catkin_package(
LIBRARIES 
ras_project_camera
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
)
catkin_python_setup()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(color_detection src/color_detection.cpp)
add_executable(camera_pcl src/camera_pcl.cpp)
add_executable(detection src/detection.cpp)
#add_executable(object_coord src/object_coord.cpp)
add_executable(camera_tf_node src/camera_transform.cpp)
target_link_libraries(color_detection ${catkin_LIBRARIES})
target_link_libraries(camera_pcl ${catkin_LIBRARIES})
target_link_libraries(detection ${catkin_LIBRARIES})
#target_link_libraries(object_coord ${catkin_LIBRARIES})
target_link_libraries(camera_tf_node ${catkin_LIBRARIES})
