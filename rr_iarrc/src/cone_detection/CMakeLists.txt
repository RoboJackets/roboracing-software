find_package( OpenCV REQUIRED ) # locate OpenCV in system
include_directories( ${OpenCV_INCLUDE_DIRS} ) # provide library headers

add_executable(cone_detection cone_detection.cpp)
target_link_libraries(cone_detection ${catkin_LIBRARIES})
target_link_libraries( cone_detection ${OpenCV_LIBS} )
add_dependencies(cone_detection rr_platform_generate_messages_cpp)

add_executable(cone_detection_point cone_detection_point.cpp)
target_link_libraries(cone_detection_point ${catkin_LIBRARIES})
target_link_libraries( cone_detection_point ${OpenCV_LIBS} )
add_dependencies(cone_detection_point rr_platform_generate_messages_cpp)

