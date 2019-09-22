add_executable(laplacian_line_detector laplacian_line_detector.cpp)
target_link_libraries(laplacian_line_detector ${catkin_LIBRARIES} ${OpenCV_LIBS})
