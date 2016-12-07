add_executable(road_detector road_detector.cpp)
target_link_libraries(road_detector ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
