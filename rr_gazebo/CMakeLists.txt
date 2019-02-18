cmake_minimum_required(VERSION 2.8.3)
project(rr_gazebo)

set(CMAKE_CXX_STANDARD 17)

find_package( catkin REQUIRED COMPONENTS
        roscpp
        rospy
        gazebo_ros
        rr_platform
)

# Try to find Protobuf and fix if needed
include(FindProtobuf)
find_package(Protobuf)
if(NOT Protobuf_INCLUDE_DIR)
        set(Protobuf_INCLUDE_DIR /usr/include/)
endif()

find_package(gazebo REQUIRED)

include_directories(${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${SDFormat_INCLUDE_DIRS})

catkin_package(
        DEPENDS gazebo_ros
        CATKIN_DEPENDS roscpp rr_platform
        INCLUDE_DIRS
        LIBRARIES
)

add_executable(macaroni_controller nodes/macaroni_controller.cpp)
target_link_libraries(macaroni_controller ${catkin_LIBRARIES})
add_dependencies(macaroni_controller rr_platform_generate_messages_cpp)

catkin_install_python(
        PROGRAMS
        scripts/controller_spawner.py
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS
        scripts/spawn_macaroni_platform.sh
        scripts/controller_spawner.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
