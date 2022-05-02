add_executable(udp_drive_relay udp_node.cpp)
add_library(udp_socket udp_socket.cpp)
target_link_libraries(udp_drive_relay udp_socket ${catkin_LIBRARIES})
add_dependencies(udp_drive_relay udp_socket ${catkin_EXPORTED_TARGETS})