add_executable(follower follower.h follower.cpp)
target_link_libraries(follower ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(follower rr_platform_generate_messages_cpp)
