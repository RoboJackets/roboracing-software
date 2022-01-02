cmake_minimum_required(VERSION 3.5)
project(rr_gazebo)

find_package(ament_cmake REQUIRED)
find_package(rr_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)


install(
    PROGRAMS
    nodes/imu_axes.cpp
    nodes/sim_car_controller.cpp
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
#     ${CMAKE_CURRENT_BINARY_DIR}/models/
#     DESTINATION share/${PROJECT_NAME}/models
# )
ament_package()