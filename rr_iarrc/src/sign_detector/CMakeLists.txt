add_executable(sign_detector sign_detector.cpp)
target_link_libraries(sign_detector ${catkin_LIBRARIES} )
add_dependencies(sign_detector rr_iarrc_generate_messages)
