cmake_minimum_required(VERSION 2.8.3)
project(rr_iarrc)

set(CMAKE_CXX_STANDARD 17)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        cv_bridge
        pcl_ros
        image_transport
        rr_msgs
        )

find_package(OpenCV REQUIRED)

include_directories(${catkin_INCLUDE_DIRS})

###################################
## catkin specific configuration ##
###################################
catkin_package(
        CATKIN_DEPENDS roscpp rospy rr_msgs
)

###########
## Build ##
###########

add_subdirectory(src/color_detector)
add_subdirectory(src/startlight_watcher)
add_subdirectory(src/finish_line_watcher)
add_subdirectory(src/qualify)
add_subdirectory(src/ol_reliable)
add_subdirectory(src/hsv_tuner)
add_subdirectory(src/laplacian_line_detection)
add_subdirectory(src/drag_centerline_planner)
add_subdirectory(src/cone_detection)
add_subdirectory(src/urc_controller)
add_subdirectory(src/turning_test)
add_subdirectory(src/sign_detector)
add_subdirectory(src/urc_image_merger)
