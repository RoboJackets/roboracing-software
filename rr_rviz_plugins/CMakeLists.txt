cmake_minimum_required(VERSION 3.0.2)
project(rr_rviz_plugins)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rviz
  rr_msgs
)

find_package(Qt5Widgets REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rviz rr_msgs
)

###########
## Build ##
###########


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set (rviz_plugins_SRCS
    src/ExamplePanel.cpp
    src/ResetPanel.cpp
    src/StartlightPanel.cpp
    src/ChassisPanel.cpp
)

set (rviz_plugins_HDRS
    include/rr_rviz_plugins/ExamplePanel.h
    include/rr_rviz_plugins/ResetPanel.h
    include/rr_rviz_plugins/StartlightPanel.h
    include/rr_rviz_plugins/ChassisPanel.h
)

qt5_wrap_cpp(rviz_plugins_MOCS ${rviz_plugins_HDRS})

add_library(${PROJECT_NAME} ${rviz_plugins_SRCS} ${rviz_plugins_MOCS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} Qt5::Widgets)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

install(FILES
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)