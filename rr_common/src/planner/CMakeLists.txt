add_library(distance_checker distance_checker.cpp distance_checker.h)
target_link_libraries(distance_checker ${catkin_LIBRARIES})

add_library(kinematic_bicycle_model kinematic_bicycle_model.cpp kinematic_bicycle_model.h)
target_link_libraries(kinematic_bicycle_model ${catkin_LIBRARIES})

add_library(random_sample_planner random_sample_planner.cpp random_sample_planner.h planner.h)
target_link_libraries(random_sample_planner ${catkin_LIBRARIES})

add_executable(planner planner_node.cpp)
target_link_libraries(planner random_sample_planner ${catkin_LIBRARIES})
add_dependencies(planner rr_platform_generate_messages_cpp)
