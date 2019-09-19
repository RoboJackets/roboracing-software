cmake_minimum_required(VERSION 2.8.3)
project(rr_common)

set(CMAKE_CXX_STANDARD 17)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        cv_bridge
        pcl_ros
        image_transport
        sound_play
        rr_msgs
        )

#find_package(OpenCV REQUIRED)
#find_package(PCL REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
        INCLUDE_DIRS include
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
        ${rr_platform_INCLUDE_DIRS}
)

add_library(rr_common src/image_flipper/image_flipper.cpp)
target_link_libraries(rr_common ${catkin_LIBRARIES})

add_subdirectory(src/planner)
add_subdirectory(src/image_pcl_converter)
add_subdirectory(src/pointcloud_combiner)
add_subdirectory(src/navigation_controller)
add_subdirectory(src/sound_node)
add_subdirectory(src/follower)
add_subdirectory(src/rosbag_remote_recorder)
add_subdirectory(src/local_mapper)
add_subdirectory(src/pose_tracker)
