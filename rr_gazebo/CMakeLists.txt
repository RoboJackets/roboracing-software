cmake_minimum_required(VERSION 2.8.3)
project(rr_gazebo)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        gazebo_ros
        rr_msgs
)

# Try to find Protobuf and fix if needed
#include(FindProtobuf)
#find_package(Protobuf)
#if (NOT Protobuf_INCLUDE_DIR)
#    set(Protobuf_INCLUDE_DIR /usr/include/)
#endif ()

#include_directories(${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${SDFormat_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
        DEPENDS gazebo_ros
        CATKIN_DEPENDS roscpp
        INCLUDE_DIRS
        LIBRARIES
)

add_executable(macaroni_controller nodes/macaroni_controller.cpp)
target_link_libraries(macaroni_controller ${catkin_LIBRARIES})
add_dependencies(macaroni_controller ${catkin_EXPORTED_TARGETS})

add_executable(imu_axes nodes/imu_axes.cpp)
target_link_libraries(imu_axes ${catkin_LIBRARIES})
add_dependencies(imu_axes ${catkin_EXPORTED_TARGETS})

catkin_install_python(
        PROGRAMS
        scripts/controller_spawner.py
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS
        scripts/spawn_macaroni_platform.sh
        scripts/controller_spawner.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
