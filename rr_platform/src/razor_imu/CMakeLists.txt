add_executable(razor_imu razor_imu.cpp)
target_link_libraries(razor_imu ${catkin_LIBRARIES} rr_serial)
add_dependencies(razor_imu rr_platform_generate_messages_cpp)
