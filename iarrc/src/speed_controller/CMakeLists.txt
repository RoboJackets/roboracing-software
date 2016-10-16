add_executable(speed_controller speed_controller.cpp)
target_link_libraries(speed_controller ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(speed_controller  rr_platform_generate_messages_cpp)
