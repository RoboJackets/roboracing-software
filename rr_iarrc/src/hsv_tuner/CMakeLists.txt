add_executable(hsv_tuner hsv_tuner.cpp)
target_link_libraries(hsv_tuner ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(hsv_tuner ${catkin_EXPORTED_TARGETS})
