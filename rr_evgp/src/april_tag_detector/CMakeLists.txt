add_executable(april_tag_detector april_tag_detector.cpp)
target_link_libraries(april_tag_detector ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(april_tag_detector ${catkin_EXPORTED_TARGETS})
