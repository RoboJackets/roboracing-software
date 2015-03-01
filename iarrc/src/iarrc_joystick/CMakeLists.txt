add_executable(iarrc_joystick iarrc_joystick.cpp)
target_link_libraries(iarrc_joystick ${catkin_LIBRARIES})
add_dependencies(iarrc_joystick ${iarrc_msgs_EXPORTED_TARGETS})
