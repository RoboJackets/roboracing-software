add_executable(histogram histogram.cpp)
target_link_libraries(histogram ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
