cmake_minimum_required(VERSION 2.8.3)
project(iarrc_motor_relay)
SET(CMAKE_BUILD_TYPE "Release")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS message_generation roscpp sensor_msgs std_msgs iarrc_msgs)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS message_runtime sensor_msgs std_msgs iarrc_msgs
  DEPENDS system_lib)

###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS})

add_executable(iarrc_motor_relay_node src/iarrc_motor_relay_node.cpp)
add_dependencies(iarrc_motor_relay_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(iarrc_motor_relay_node ${catkin_LIBRARIES})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS iarrc_motor_relay_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
