add_executable(urc_turn_action urc_turn_action.cpp)
target_link_libraries(urc_turn_action ${catkin_LIBRARIES})
add_dependencies(urc_turn_action ${catkin_EXPORTED_TARGETS})