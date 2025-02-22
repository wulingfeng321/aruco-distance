cmake_minimum_required(VERSION 3.0.2)
project(aruco_scan)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  custom_msgs
)
#find_package(OpenCV REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp rospy sensor_msgs cv_bridge
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(aruco_scan
  src/main.cpp
) 

target_link_libraries(aruco_scan
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

