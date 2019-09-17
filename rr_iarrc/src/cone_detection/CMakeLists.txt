find_package( OpenCV REQUIRED ) # locate OpenCV in system
include_directories( ${OpenCV_INCLUDE_DIRS} ) # provide library headers

add_executable(cone_bottom_detector cone_bottom_detector.cpp)
target_link_libraries(cone_bottom_detector ${catkin_LIBRARIES})
target_link_libraries( cone_bottom_detector ${OpenCV_LIBS} )
add_dependencies(cone_bottom_detector rr_platform_generate_messages_cpp)

add_executable(cone_height_detector cone_height_detector.cpp)
target_link_libraries(cone_height_detector ${catkin_LIBRARIES})
target_link_libraries( cone_height_detector ${OpenCV_LIBS} )
add_dependencies(cone_height_detector rr_platform_generate_messages_cpp)


