cmake_minimum_required(VERSION 3.0.2)
project(rr_common)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-Wall")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        cv_bridge
        pcl_ros
        image_transport
        rr_msgs
        costmap_2d
        tf
        tf2_geometry_msgs
        parameter_assertions
        dynamic_reconfigure
        )

find_package(OpenCV REQUIRED)

##########################
## dynamic reconfigure  ##
##########################

generate_dynamic_reconfigure_options(
        cfg/planner_configuration.cfg
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
        INCLUDE_DIRS include
        LIBRARIES rr_color_filter
        CATKIN_DEPENDS roscpp rospy std_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_library(rr_common src/image_flipper/image_flipper.cpp)
target_link_libraries(rr_common ${catkin_LIBRARIES})

add_subdirectory(src/planner)
add_subdirectory(src/image_pcl_converter)
add_subdirectory(src/pointcloud_combiner)
add_subdirectory(src/navigation_controller)
add_subdirectory(src/follower)
add_subdirectory(src/rosbag_remote_recorder)
add_subdirectory(src/local_mapper)
add_subdirectory(src/pose_tracker)
add_subdirectory(src/camera_geometry)
add_subdirectory(src/image_transformation)
add_subdirectory(src/color_filter)
