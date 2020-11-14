add_library(tag_detection tag_detection.cpp detection.cpp)
target_link_libraries(tag_detection ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(tag_detection ${catkin_EXPORTED_TARGETS})
