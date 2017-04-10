add_library(path_lib path_structs.cpp density_cluster.cpp)
add_executable(planner planner.cpp)
target_link_libraries(planner path_lib ${catkin_LIBRARIES})
add_dependencies(planner rr_platform_generate_messages_cpp)
