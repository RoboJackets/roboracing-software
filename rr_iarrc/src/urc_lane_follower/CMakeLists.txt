find_package( OpenCV REQUIRED ) # locate OpenCV in system
include_directories( ${OpenCV_INCLUDE_DIRS} ) # provide library headers

add_executable(urc_lane_follower urc_lane_follower.cpp)
target_link_libraries(urc_lane_follower ${catkin_LIBRARIES})
target_link_libraries( urc_lane_follower ${OpenCV_LIBS} )
add_dependencies(urc_lane_follower rr_platform_generate_messages_cpp)

add_executable(urc_center_keeper urc_center_keeper.cpp)
target_link_libraries(urc_center_keeper ${catkin_LIBRARIES})
target_link_libraries( urc_center_keeper ${OpenCV_LIBS} )
add_dependencies(urc_center_keeper rr_platform_generate_messages_cpp)
