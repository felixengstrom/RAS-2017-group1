cmake_minimum_required(VERSION 2.8.3)
project(ras_project_odometry)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
catkin_package()
## is used, also find other catkin packages
include_directories(
    # include
      ${catkin_INCLUDE_DIRS}
      )

find_package(catkin REQUIRED
             roscpp
             std_msgs
             tf
             phidgets)

add_executable(odometry_node src/odometry.cpp)
target_link_libraries(odometry_node ${catkin_LIBRARIES}) 
