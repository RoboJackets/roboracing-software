cmake_minimum_required(VERSION 2.8.3)
project(rr_description)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

## Find catkin macros and libraries
find_package(
    catkin REQUIRED COMPONENTS
    gencpp
    genpy
    roscpp
    rospy
    geometry_msgs
    sensor_msgs
    rr_platform
    tf
)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")


## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(
    CATKIN_DEPENDS roscpp rospy message_runtime
    DEPENDS system_lib
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(speedi_joint_updater speedi/speedi_joint_updater.cpp)
target_link_libraries(speedi_joint_updater ${catkin_LIBRARIES})
add_dependencies(speedi_joint_updater rr_platform_generate_messages_cpp)
