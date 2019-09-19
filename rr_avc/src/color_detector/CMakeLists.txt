add_executable(color_detector_avc color_detector_avc.cpp)
target_link_libraries(color_detector_avc ${catkin_LIBRARIES})
add_dependencies(color_detector_avc ${catkin_EXPORTED_TARGETS})
