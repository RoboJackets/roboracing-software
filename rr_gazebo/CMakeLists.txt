cmake_minimum_required(VERSION 2.8.3)
project(rr_gazebo)

find_package(
        catkin
        REQUIRED COMPONENTS
        gazebo_ros
)

find_package(gazebo REQUIRED)

include_directories(${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${SDFormat_INCLUDE_DIRS})

catkin_package(
        DEPENDS
        gazebo_ros
        CATKIN_DEPENDS
        INCLUDE_DIRS
        LIBRARIES
)
