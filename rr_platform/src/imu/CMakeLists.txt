add_executable(imu imu.cpp)
target_link_libraries(imu ${catkin_LIBRARIES} SerialPort)