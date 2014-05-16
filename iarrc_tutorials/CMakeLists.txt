cmake_minimum_required(VERSION 2.8.3)
project(iarrc_tutorials)

SET(CMAKE_BUILD_TYPE "Debug")
set(BUILD_FLAGS "-std=c++0x -Wreturn-type -Wno-enum-compare")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS message_generation roscpp sensor_msgs std_msgs iarrc_msgs tf cv_bridge pcl_ros pcl)

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

#######################################
## Declare ROS messages and services ##
#######################################

## Generate actions in the 'action' folder
# add_action_files(
#   DIRECTORY
#   action
#   FILES
#   LaserScan.action
# )

## Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY
  msg
  FILES
  Empty.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   DIRECTORY
#   FILES
#   LidarAggregation.srv
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES iarrc_tutorials
  CATKIN_DEPENDS message_runtime sensor_msgs std_msgs iarrc_msgs tf cv_bridge pcl_ros pcl
  DEPENDS system_lib QtCore QtGui)

######################
## Qt configuration ##
######################
# Run files through "moc", Qt's meta-object compiler
qt4_wrap_cpp(MOC_FILES
  include/iarrc_tutorials/iarrc_panel.h)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS})
link_libraries(${catkin_LIBRARIES})

## Collect the source files
file(GLOB lib_source "src/libraries/*.cpp")
set(lib_source ${lib_source} ${MOC_FILES}) # Add Qt auto-generated files to library
list(SORT lib_source)

file(GLOB nodes_source "src/nodes/*.cpp")
list(SORT nodes_source)

file(GLOB utils_source "src/utils/*.cpp")
list(SORT utils_source)

file(GLOB plugins_source "src/plugins/*.cpp")
set(plugins_source ${plugins_source} ${MOC_FILES}) # Add Qt auto-generated files to plugins
list(SORT plugins_source)

file(GLOB include_source "include/iarrc_tutorials/*.h")
list(SORT include_source)

## Declare a cpp library
add_library(${PROJECT_NAME} SHARED ${lib_source})
# add_dependencies(${PROJECT_NAME} hubo_gt_msgs_gencpp)
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS ${BUILD_FLAGS})
add_library(${PROJECT_NAME}_rviz SHARED ${plugins_source} ${include_source})
# add_dependencies(${PROJECT_NAME}_rviz hubo_gt_msgs_gencpp)
set_target_properties(${PROJECT_NAME}_rviz PROPERTIES COMPILE_FLAGS ${BUILD_FLAGS})

# Build executables
message(STATUS "\n-- ROS NODES: ")
foreach(node_src_file ${nodes_source})
	get_filename_component(node_base ${node_src_file} NAME_WE)
	message(STATUS "Adding ROS Node ${node_base}" )
		add_executable(${node_base} ${node_src_file} ${include_source})
	target_link_libraries(${node_base} ${PROJECT_NAME})
	add_dependencies(${node_base} hubo_gt_msgs_gencpp)
	set_target_properties(${node_base} PROPERTIES COMPILE_FLAGS ${BUILD_FLAGS})
endforeach(node_src_file)

# Build utilities
message(STATUS "\n-- UTILITY SCRIPTS: ")
foreach(util_src_file ${utils_source})
	get_filename_component(util_base ${util_src_file} NAME_WE)
	message(STATUS "Adding ROS util ${util_base}" )
	add_executable(${util_base} ${util_src_file} ${include_source})
	target_link_libraries(${util_base} ${PROJECT_NAME})
	add_dependencies(${util_base} hubo_gt_msgs_gencpp)
	set_target_properties(${util_base} PROPERTIES COMPILE_FLAGS ${BUILD_FLAGS})
endforeach(util_src_file)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS iarrc_tutorial_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
