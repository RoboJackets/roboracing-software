add_library(rr_evgp center_path_finder.cpp)
target_link_libraries(rr_evgp ${catkin_LIBRARIES})
add_dependencies(rr_iarrc ${catkin_EXPORTED_TARGETS})
