add_executable(color_detector_circuit color_detector_circuit.cpp)
target_link_libraries(color_detector_circuit ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
