find_package( OpenCV REQUIRED ) # locate OpenCV in system
include_directories( ${OpenCV_INCLUDE_DIRS} ) # provide library headers

add_executable(urc_image_merger urc_image_merger.cpp)
target_link_libraries(urc_image_merger ${catkin_LIBRARIES})
target_link_libraries( urc_image_merger ${OpenCV_LIBS} )
add_dependencies(urc_image_merger rr_platform_generate_messages_cpp)

