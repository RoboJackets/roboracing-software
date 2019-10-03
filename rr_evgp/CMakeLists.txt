cmake_minimum_required(VERSION 2.8.3)
project(rr_evgp)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        rr_common
        rr_description
        rr_gazebo
        rr_platform
        sensor_msgs
        geometry_msgs
        costmap_2d
        parameter_assertions
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rr_evgp
#  CATKIN_DEPENDS geometry_msgs roscpp rospy rr_common rr_description rr_gazebo rr_platform sensor_msgs
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_library(rr_evgp_plugins src/test_layer.cpp)
target_link_libraries(rr_evgp_plugins ${catkin_LIBRARIES})
