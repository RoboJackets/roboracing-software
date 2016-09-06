cmake_minimum_required(VERSION 2.8.3)
project(avc)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  rr_platform_msgs
  cv_bridge
  pcl_ros
  sick_tim
  laser_geometry
  image_transport
)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED filters)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES iarrc_software
  CATKIN_DEPENDS roscpp rospy std_msgs rr_platform_msgs
  DEPENDS system_lib
)

###########
## Build ##
###########

add_executable(color_detector_avc src/color_detector_avc.cpp)
target_link_libraries(color_detector_avc ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

## Specify additional locations of header files
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

add_subdirectory(src/planner)
