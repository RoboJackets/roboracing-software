add_executable(imu imu.cpp)
target_link_libraries(imu ${catkin_LIBRARIES} rr_serial)
add_dependencies(imu rr_platform_generate_messages_cpp)
