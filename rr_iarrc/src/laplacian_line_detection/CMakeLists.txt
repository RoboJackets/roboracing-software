find_package( OpenCV REQUIRED ) # locate OpenCV in system
include_directories( ${OpenCV_INCLUDE_DIRS} ) # provide library headers

add_executable(laplacian_line_detector laplacian_line_detector.cpp)
target_link_libraries(laplacian_line_detector ${catkin_LIBRARIES})
target_link_libraries( laplacian_line_detector ${OpenCV_LIBS} )
