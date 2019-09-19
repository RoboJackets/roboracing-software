add_executable(urc_controller urc_controller.cpp)
target_link_libraries(urc_controller ${catkin_LIBRARIES})
add_dependencies(urc_controller ${catkin_EXPORTED_TARGETS})
