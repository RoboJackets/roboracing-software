cmake_minimum_required(VERSION 2.8.3)
project(rr_common)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

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
    sound_play
)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

###################################
## catkin specific configuration ##
###################################
catkin_package(
  #INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
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

add_library(rr_common src/image_flipper/image_flipper.cpp)
target_link_libraries(rr_common ${catkin_LIBRARIES})

add_subdirectory(src/planner)
add_subdirectory(src/image_pcl_converter)
add_subdirectory(src/mapper)
add_subdirectory(src/navigation_controller)
add_subdirectory(src/sound_node)
add_subdirectory(src/follower)
