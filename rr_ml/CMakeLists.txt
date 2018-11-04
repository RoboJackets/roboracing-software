cmake_minimum_required(VERSION 2.8.3)
project(rr_ml)

#set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

#list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
        rospy
        std_msgs
        rr_platform
        cv_bridge
        )

find_package(OpenCV REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
        CATKIN_DEPENDS rospy std_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
        nodes

        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
