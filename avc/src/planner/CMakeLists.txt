add_executable(planner planner.cpp)
target_link_libraries(planner ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
