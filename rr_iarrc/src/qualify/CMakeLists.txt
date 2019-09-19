add_executable(qualify qualify.cpp)
target_link_libraries(qualify ${catkin_LIBRARIES})
add_dependencies(qualify ${catkin_EXPORTED_TARGETS})
