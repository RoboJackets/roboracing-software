cmake_minimum_required(VERSION 3.8)
project(rr_util)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
# find dependencies
find_package(ament_cmake REQUIRED)

# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclpy REQUIRED)
find_package(rr_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)


add_executable(joystick_driver src/joystick_driver/joystick_driver.cpp)
ament_target_dependencies(joystick_driver
        rclcpp
        sensor_msgs
        rclcpp_components
        rr_msgs
)

install(
    TARGETS
    joystick_driver
    DESTINATION lib/${PROJECT_NAME}
)

install(
    PROGRAMS
    scripts/udp_pub.py
    scripts/udp_sub.py
    scripts/namespace_hack.py
    DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY
    src
    launch
    scripts
    DESTINATION share/${PROJECT_NAME}
)

###########
## Build ##
###########

ament_package()
