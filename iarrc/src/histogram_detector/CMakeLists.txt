find_package( OpenCV REQUIRED ) # locate OpenCV in system
include_directories( ${OpenCV_INCLUDE_DIRS} ) # provide library headers


add_executable(canny_line_detector canny_line_detector.cpp)
target_link_libraries(canny_line_detector ${catkin_LIBRARIES})
add_dependencies(canny_line_detector rr_platform_generate_messages_cpp)
target_link_libraries( canny_line_detector ${OpenCV_LIBS} )

add_executable(Cone_detector Cone_detector.cpp)
target_link_libraries(Cone_detector ${catkin_LIBRARIES})
add_dependencies(Cone_detector rr_platform_generate_messages_cpp)
target_link_libraries( Cone_detector ${OpenCV_LIBS} )

#add_executable(histogram_detector histogram_detector.cpp)
#target_link_libraries(histogram_detector ${catkin_LIBRARIES})
#add_dependencies(histogram_detector rr_platform_generate_messages_cpp)



