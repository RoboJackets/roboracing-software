find_package( OpenCV REQUIRED ) # locate OpenCV in system
include_directories( ${OpenCV_INCLUDE_DIRS} ) # provide library headers

add_executable(cone_detection_bottom cone_detection_bottom.cpp)
target_link_libraries(cone_detection_bottom ${catkin_LIBRARIES})
target_link_libraries( cone_detection_bottom ${OpenCV_LIBS} )
add_dependencies(cone_detection_bottom rr_platform_generate_messages_cpp)

add_executable(cone_detection_height cone_detection_height.cpp)
target_link_libraries(cone_detection_height ${catkin_LIBRARIES})
target_link_libraries( cone_detection_height ${OpenCV_LIBS} )
add_dependencies(cone_detection_height rr_platform_generate_messages_cpp)

