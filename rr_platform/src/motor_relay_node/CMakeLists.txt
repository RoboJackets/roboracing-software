add_executable(motor_relay_node motor_relay_node.cpp)
target_link_libraries(motor_relay_node ${catkin_LIBRARIES})
add_dependencies(motor_relay_node ${iarrc_msgs_EXPORTED_TARGETS})
