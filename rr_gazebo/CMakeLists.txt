cmake_minimum_required(VERSION 3.5)
project(rr_gazebo)

find_package(ament_cmake REQUIRED)
find_package(rr_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(sim_car_controller SHARED nodes/sim_car_controller.cpp)
ament_target_dependencies(sim_car_controller
    "rclcpp"
    "rr_msgs"
    "geometry_msgs"
    "nav_msgs"
    "sensor_msgs"
    "std_msgs"
)

add_library(imu_axes SHARED nodes/imu_axes.cpp)
ament_target_dependencies(imu_axes
    "rclcpp"
    "rr_msgs"
    "sensor_msgs"
)

install(
    TARGETS
    imu_axes
    sim_car_controller
    DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY
    launch
    models
    worlds
    config
    DESTINATION share/${PROJECT_NAME}
)

# not sure if I need this or what this does
# install(
#     DIRECTORY
#     ${CMAKE_C_DIR}/models/
#     DESTINATION share/${PROJECT_NAME}/models
# )

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/rr_gazebo.dsv.in")

ament_package()