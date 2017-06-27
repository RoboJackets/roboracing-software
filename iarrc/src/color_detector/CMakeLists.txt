add_executable(color_detector color_detector.cpp)
target_link_libraries(color_detector ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
