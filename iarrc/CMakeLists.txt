cmake_minimum_required(VERSION 2.8.3)
project(iarrc)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  iarrc_msgs
  cv_bridge
  pcl_ros
  sick_tim
  laser_geometry
)

find_package(OpenCV REQUIRED)
find_package(Eigen REQUIRED)
find_package(PCL REQUIRED filters)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES iarrc_software
  CATKIN_DEPENDS roscpp rospy std_msgs iarrc_msgs
  DEPENDS system_lib
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

add_subdirectory(src/circuit_race_steerer)
add_subdirectory(src/drag_race_end_detector)
add_subdirectory(src/drag_race_steerer)
add_subdirectory(src/frame_merger)
add_subdirectory(src/iarrc_circuit_race_controller)
add_subdirectory(src/iarrc_cone_detector)
add_subdirectory(src/iarrc_dragrace_controller)
add_subdirectory(src/iarrc_joystick)
add_subdirectory(src/iarrc_line_detection)
add_subdirectory(src/iarrc_motor_relay_node)
add_subdirectory(src/iarrc_scan_to_cloud)
add_subdirectory(src/stoplight_watcher)
add_subdirectory(src/visual_cone_detector)
add_subdirectory(src/frame_inverted)
add_subdirectory(src/pointcloud_to_image)
add_subdirectory(src/projection)
add_subdirectory(src/iarrc_world_model)
add_subdirectory(src/iarrc_planner)


