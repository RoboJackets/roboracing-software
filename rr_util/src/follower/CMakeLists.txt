add_executable(follower follower.h follower.cpp)
target_link_libraries(follower ${catkin_LIBRARIES})
add_dependencies(follower ${catkin_EXPORTED_TARGETS})
