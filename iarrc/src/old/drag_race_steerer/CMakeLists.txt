add_executable(drag_race_steerer drag_race_steerer.cpp)
target_link_libraries(drag_race_steerer ${catkin_LIBRARIES})
add_dependencies(drag_race_steerer ${iarrc_msgs_EXPORTED_TARGETS})
