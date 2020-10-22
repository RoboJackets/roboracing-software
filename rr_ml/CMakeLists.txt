cmake_minimum_required(VERSION 3.0.2)
project(rr_ml)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
        rospy
        rr_msgs
        cv_bridge
        )

#find_package(OpenCV REQUIRED)

catkin_package(
        CATKIN_DEPENDS rospy rr_msgs
)

add_subdirectory(nodes)
