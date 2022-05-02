add_library(udp_socket udp_socket.cpp)
add_library(udp_sock udp_sock.cpp)

add_executable(udp_drive_relay udp_node.cpp)
target_link_libraries(udp_drive_relay udp_socket ${catkin_LIBRARIES})
add_dependencies(udp_drive_relay udp_socket ${catkin_EXPORTED_TARGETS})

add_executable(udp_sub_cpp udp_sub.cpp)
target_link_libraries(udp_sub_cpp udp_sock ${catkin_LIBRARIES})
add_dependencies(udp_sub_cpp udp_sock ${catkin_EXPORTED_TARGETS})

add_executable(udp_pub_cpp udp_pub.cpp)
target_link_libraries(udp_pub_cpp udp_sock ${catkin_LIBRARIES})
add_dependencies(udp_pub_cpp udp_sock ${catkin_EXPORTED_TARGETS})