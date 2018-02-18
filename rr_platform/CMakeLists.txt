cmake_minimum_required(VERSION 2.8.3)
project(rr_platform)

set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS}")

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

## Find catkin macros and libraries
find_package(
    catkin REQUIRED COMPONENTS
    gencpp
    genpy
    roscpp
    rospy
    std_msgs
    sensor_msgs
    cv_bridge
    pcl_ros
    sick_tim
    laser_geometry
    image_transport
    std_msgs
    message_generation
    geometry_msgs
)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED filters)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

## Generate messages in the 'msg' folder
add_message_files(
    FILES
    speed.msg
    steering.msg
    camera_pose.msg
    chassis_state.msg
)

## Generate services in the 'srv' folder
add_service_files(
    FILES
    transform_image.srv
    calibrate_image.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
    sensor_msgs
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_rostest_gtest(test_joystick_driver test/joystick_driver/test_joystick_driver.test test/joystick_driver/test_joystick_driver.cpp)
    target_link_libraries(test_joystick_driver ${catkin_LIBRARIES})
    add_dependencies(test_joystick_driver rr_platform_generate_messages_cpp)
    add_rostest_gtest(test_scanToPointCloud test/scanToPointCloud/test_scanToPointCloud.test test/scanToPointCloud/test_scanToPointCloud.cpp)
    target_link_libraries(test_scanToPointCloud ${catkin_LIBRARIES})
    add_dependencies(test_scanToPointCloud rr_platform_generate_messages_cpp)
endif()

## Specify additional locations of header files
include_directories(
    include
    ${PCL_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
)

add_subdirectory(src/joystick_driver)
add_subdirectory(src/motor_relay_node)
add_subdirectory(src/bigoli_motor_relay_node)
add_subdirectory(src/camera)
add_subdirectory(src/scanToPointCloud)
add_subdirectory(src/mpu9250_imu)
add_subdirectory(src/ultrasonic_array)

install(PROGRAMS launch/bigoli_camera_dynconfig.sh
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
