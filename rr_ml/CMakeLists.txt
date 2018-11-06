cmake_minimum_required(VERSION 2.8.3)
project(rr_ml)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
        rospy
        std_msgs
        rr_platform
        cv_bridge
        )

find_package(OpenCV REQUIRED)

catkin_package(
        CATKIN_DEPENDS rospy std_msgs rr_platform
)

add_subdirectory(nodes)
