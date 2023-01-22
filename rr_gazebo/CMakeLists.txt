cmake_minimum_required(VERSION 3.5)
project(rr_gazebo)

find_package(ament_cmake REQUIRED)
find_package(rr_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)

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

ament_package()
