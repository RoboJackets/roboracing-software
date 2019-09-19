add_executable(razor_imu razor_imu.cpp)
target_link_libraries(razor_imu ${catkin_LIBRARIES} rr_serial)
