add_executable(planner planner.cpp)
target_link_libraries(planner ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(planner rr_platform_msgs_generate_messages_cpp)
