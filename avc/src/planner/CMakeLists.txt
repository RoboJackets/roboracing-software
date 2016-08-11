add_executable(planner planner.cpp path.cpp)
target_link_libraries(planner ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
