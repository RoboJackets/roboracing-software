cmake_minimum_required(VERSION 2.8.3)
project(iarrc)

SET(CMAKE_BUILD_TYPE "Debug")
set(BUILD_FLAGS "-std=c++0x -Wreturn-type -Wno-enum-compare")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS iarrc_msgs roscpp sensor_msgs std_msgs tf cv_bridge)

## Include Eigen
find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

## Include Qt
find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})

# Include the OpenCV library
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIR})
link_libraries(${OpenCV_LIBS})

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS message_runtime sensor_msgs std_msgs iarrc_msgs tf cv_bridge
  DEPENDS system_lib QtCore QtGui)

###########
## Build ##
###########

## Collect the source files

file(GLOB nodes_source "src/nodes/*.cpp")
list(SORT nodes_source)

# Build executables
message(STATUS "\n-- ROS NODES: ")
foreach(node_src_file ${nodes_source})
	get_filename_component(node_base ${node_src_file} NAME_WE)
	message(STATUS "Adding ROS Node ${node_base}" )
	add_executable(${node_base} ${node_src_file} ${include_source})
	add_dependencies(${node_base} hubo_gt_msgs_gencpp)
	set_target_properties(${node_base} PROPERTIES COMPILE_FLAGS ${BUILD_FLAGS})
	target_link_libraries(${node_base} ${catkin_LIBRARIES})
	add_dependencies(${node_base} iarrc_msgs_gencpp)
endforeach(node_src_file)
