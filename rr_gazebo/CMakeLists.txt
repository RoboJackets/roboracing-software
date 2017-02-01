cmake_minimum_required(VERSION 2.8.3)
project(rr_gazebo)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package( catkin REQUIRED COMPONENTS
        roscpp
        rospy
        gazebo_ros
        rr_platform
)

find_package(gazebo REQUIRED)

include_directories(${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${SDFormat_INCLUDE_DIRS})

catkin_package(
        DEPENDS gazebo_ros
        CATKIN_DEPENDS roscpp rr_platform
        INCLUDE_DIRS
        LIBRARIES
)

add_executable(speedi_controller nodes/speedi_controller.cpp)
target_link_libraries(speedi_controller ${catkin_LIBRARIES})
add_dependencies(speedi_controller rr_platform_generate_messages_cpp)

catkin_install_python(
        PROGRAMS
        scripts/controller_spawner.py
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS
        scripts/spawn_speedi_platform.sh
        scripts/controller_spawner.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})