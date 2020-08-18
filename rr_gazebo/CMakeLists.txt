cmake_minimum_required(VERSION 3.0.2)
project(rr_gazebo)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        gazebo_ros
        rr_msgs
        parameter_assertions
)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
        DEPENDS gazebo_ros
        CATKIN_DEPENDS roscpp
        INCLUDE_DIRS
        LIBRARIES
)

add_executable(sim_car_controller nodes/sim_car_controller.cpp)
target_link_libraries(sim_car_controller ${catkin_LIBRARIES})
add_dependencies(sim_car_controller ${catkin_EXPORTED_TARGETS})

add_executable(imu_axes nodes/imu_axes.cpp)
target_link_libraries(imu_axes ${catkin_LIBRARIES})
add_dependencies(imu_axes ${catkin_EXPORTED_TARGETS})
