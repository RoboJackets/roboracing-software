cmake_minimum_required(VERSION 2.8.3)
project(rr_description)

find_package(
        catkin
        REQUIRED COMPONENTS
)

find_package(gazebo REQUIRED)

catkin_package(
        DEPENDS
        CATKIN_DEPENDS
        INCLUDE_DIRS
        LIBRARIES
)
