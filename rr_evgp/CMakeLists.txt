cmake_minimum_required(VERSION 3.0.2)
project(rr_evgp)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        rr_msgs
        sensor_msgs
        geometry_msgs
        costmap_2d
        parameter_assertions
        pcl_ros
        pcl_conversions
        visualization_msgs
        tf_conversions
)

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES rr_evgp
#  CATKIN_DEPENDS geometry_msgs roscpp rospy rr_common rr_description rr_gazebo rr_platform sensor_msgs costmap_2d
  CATKIN_DEPENDS
    apriltag_ros
  LIBRARIES UniformCostSearch
  DEPENDS OpenCV
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

###########
## Build ##
###########

add_subdirectory(src/opponent_detection)
add_subdirectory(src/costmap_plugins)
add_subdirectory(src/UniformCostSearch)
add_subdirectory(src/april_tag_detection)
