add_executable(joystick joystick.cpp)
target_link_libraries(joystick ${catkin_LIBRARIES})
add_dependencies(joystick ${iarrc_msgs_EXPORTED_TARGETS})
