add_library(rr_color_filter color_filter.cpp)
target_link_libraries(rr_color_filter ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(rr_color_filter ${catkin_EXPORTED_TARGETS})

add_executable(rr_color_filter_test_node color_filter_test_node.cpp)
target_link_libraries(rr_color_filter_test_node rr_color_filter ${catkin_LIBRARIES})
