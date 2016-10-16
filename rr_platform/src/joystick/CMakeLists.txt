add_executable(joystick joystick.cpp)
target_link_libraries(joystick ${catkin_LIBRARIES})
add_dependencies(joystick rr_platform_generate_messages_cpp)
