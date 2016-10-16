cmake_minimum_required(VERSION 2.8.3)
project(avc)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

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
find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRs})
add_definitions(${PCL_DEFINITIONS})

## Generate services in the 'srv' folder
add_service_files(
   FILES
   transform_image.srv
   calibrate_image.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
   sensor_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs rr_platform message_runtime
  DEPENDS system_lib
)

###########
## Build ##
###########

add_executable(color_detector_avc src/color_detector_avc.cpp)
target_link_libraries(color_detector_avc ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(color_detector_avc avc_generate_messages_cpp)

add_executable(image_transform src/image_transform.cpp)
target_link_libraries(image_transform ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(image_transform avc_generate_messages_cpp)

add_executable(trigger_calibration src/trigger_calibration.cpp)
target_link_libraries(trigger_calibration ${catkin_LIBRARIES})
add_dependencies(trigger_calibration avc_generate_messages_cpp)

## Specify additional locations of header files
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

add_subdirectory(src/planner)
add_subdirectory(src/scanToPointCloud)
add_subdirectory(src/mapper)
