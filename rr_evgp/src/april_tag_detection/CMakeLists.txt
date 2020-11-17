add_executable(tag_detection tag_detection.cpp)
target_link_libraries(tag_detection ${catkin_LIBRARIES} ${OpenCV_LIBS})