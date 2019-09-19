add_executable(follower follower.h follower.cpp)
target_link_libraries(follower ${catkin_LIBRARIES} ${PCL_LIBRARIES})
