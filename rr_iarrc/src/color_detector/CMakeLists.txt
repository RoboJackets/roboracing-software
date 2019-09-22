add_library(rr_iarrc color_detector.cpp)
target_link_libraries(rr_iarrc ${catkin_LIBRARIES})
add_dependencies(rr_iarrc ${catkin_EXPORTED_TARGETS})
