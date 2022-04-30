cmake_minimum_required(VERSION 3.0.2)
project(rr_platform)

set(CMAKE_CXX_STANDARD 17)

## Find catkin macros and libraries
find_package(catkin
        REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        sensor_msgs
        cv_bridge
        pcl_ros
        laser_geometry
        image_transport
        geometry_msgs
        rr_msgs
        )

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)


catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS roscpp rospy rr_msgs sensor_msgs geometry_msgs
)

if (CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_rostest_gtest(test_joystick_driver test/joystick_driver/test_joystick_driver.test test/joystick_driver/test_joystick_driver.cpp)
    target_link_libraries(test_joystick_driver ${catkin_LIBRARIES})
    add_dependencies(test_joystick_driver ${catkin_EXPORTED_TARGETS})

    add_rostest_gtest(test_scanToPointCloud test/scanToPointCloud/test_scanToPointCloud.test test/scanToPointCloud/test_scanToPointCloud.cpp)
    target_link_libraries(test_scanToPointCloud ${catkin_LIBRARIES})
endif ()

## Specify additional locations of header files
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)

add_library(rr_serial src/SerialPort.cpp)
add_library(rr_udp src/udp_serial.cpp)

add_subdirectory(src/joystick_driver)
add_subdirectory(src/motor_relay_node)
add_subdirectory(src/bigoli_motor_relay_node)
add_subdirectory(src/scanToPointCloud)
add_subdirectory(src/razor_imu)
add_subdirectory(src/ultrasonic_array)
add_subdirectory(src/sedani_motor_relay_node)
add_subdirectory(src/udp_socket)
