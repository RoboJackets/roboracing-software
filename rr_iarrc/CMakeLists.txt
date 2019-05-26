cmake_minimum_required(VERSION 2.8.3)
project(rr_iarrc)

set(CMAKE_CXX_STANDARD 17)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  rr_platform
  cv_bridge
  pcl_ros
  sick_tim
  laser_geometry
  image_transport
  message_generation
)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED filters)

## Generate messages in the 'msg' folder
add_message_files(
    FILES
    hsv_tuned.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs rr_platform message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)


add_subdirectory(src/color_detector)
add_subdirectory(src/stoplight_watcher)
add_subdirectory(src/finish_line_watcher)
add_subdirectory(src/qualify)
add_subdirectory(src/ol_reliable)
add_subdirectory(src/hsv_tuner)
add_subdirectory(src/laplacian_line_detection)
add_subdirectory(src/sign_detector)

