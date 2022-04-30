add_executable(udp_drive_relay udp_socket.cpp)
target_link_libraries(udp_drive_relay ${catkin_LIBRARIES} rr_udp)
add_dependencies(udp_drive_relay ${catkin_EXPORTED_TARGETS})
